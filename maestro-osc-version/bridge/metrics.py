"""Motion metric calculation and impact detection state."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any


@dataclass
class ImpactState:
    """Mutable state used by impact detection across serial frames."""

    last_impact_time: int = 0
    prev_deviation_above: bool = False
    motion_active: bool = False
    motion_enter_time: int = 0
    last_deviation_g: float = 0.0
    last_deviation_time_ms: int = 0
    interaction_active: bool = False
    interaction_start_candidate_time_ms: int = 0
    interaction_rest_candidate_time_ms: int = 0
    interaction_baseline_g: float | None = None
    interaction_stable_accel_g: float | None = None
    interaction_stable_accel_start_time_ms: int = 0
    synth_filtered_gravity: tuple[float, float, float] | None = None
    synth_horizontal_yaw_norm: float | None = None
    synth_roll_reference_quaternion: tuple[float, float, float, float] | None = None
    synth_axis_roll_deg: float = 0.0
    synth_last_roll_measurement_deg: float | None = None
    synth_last_orientation_update_ms: int = 0


def get_current_time_ms() -> int:
    """Get current time in milliseconds."""
    return int(time.time() * 1000)


def _clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a float value to a fixed range."""
    return max(min_value, min(value, max_value))


def _scale_to_unit(value: float, low: float, high: float) -> float:
    """Map value to 0..1 given low/high bounds."""
    if high <= low:
        return 1.0 if value >= high else 0.0
    return _clamp((value - low) / (high - low), 0.0, 1.0)


def _normalize_vector(x: float, y: float, z: float) -> tuple[float, float, float] | None:
    magnitude = math.sqrt(x * x + y * y + z * z)
    if magnitude <= 1e-6:
        return None
    return (x / magnitude, y / magnitude, z / magnitude)


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        (a[1] * b[2]) - (a[2] * b[1]),
        (a[2] * b[0]) - (a[0] * b[2]),
        (a[0] * b[1]) - (a[1] * b[0]),
    )


def _normalize_quaternion(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float] | None:
    magnitude = math.sqrt(sum(component * component for component in quaternion))
    if magnitude <= 1e-6:
        return None
    return tuple(component / magnitude for component in quaternion)


def _quat_conjugate(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def _quat_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz),
        (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
        (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
        (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
    )


def _rotate_vector_by_quaternion(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    vector_quaternion = (0.0, vector[0], vector[1], vector[2])
    rotated = _quat_multiply(
        _quat_multiply(quaternion, vector_quaternion),
        _quat_conjugate(quaternion),
    )
    return (rotated[1], rotated[2], rotated[3])


def _project_onto_plane(
    vector: tuple[float, float, float],
    normal: tuple[float, float, float],
) -> tuple[float, float, float] | None:
    normal_component = _dot(vector, normal)
    return _normalize_vector(
        vector[0] - (normal_component * normal[0]),
        vector[1] - (normal_component * normal[1]),
        vector[2] - (normal_component * normal[2]),
    )


def _signed_angle_about_axis(
    from_vector: tuple[float, float, float],
    to_vector: tuple[float, float, float],
    axis: tuple[float, float, float],
) -> float:
    cross_value = _cross(from_vector, to_vector)
    return math.degrees(math.atan2(_dot(axis, cross_value), _dot(from_vector, to_vector)))


def _wrap_unit_delta(current: float, previous: float) -> float:
    return ((current - previous + 0.5) % 1.0) - 0.5


def _wrap_signed_degrees(value: float) -> float:
    return ((value + 180.0) % 360.0) - 180.0


def _quaternion_relative_twist_degrees(
    reference_quaternion: tuple[float, float, float, float],
    current_quaternion: tuple[float, float, float, float],
    axis: tuple[float, float, float],
) -> float | None:
    axis_unit = _normalize_vector(*axis)
    if axis_unit is None:
        return None

    relative_quaternion = _normalize_quaternion(
        _quat_multiply(_quat_conjugate(reference_quaternion), current_quaternion)
    )
    if relative_quaternion is None:
        return None

    rw, rx, ry, rz = relative_quaternion
    vector_projection = _dot((rx, ry, rz), axis_unit)
    twist_quaternion = _normalize_quaternion(
        (
            rw,
            axis_unit[0] * vector_projection,
            axis_unit[1] * vector_projection,
            axis_unit[2] * vector_projection,
        )
    )
    if twist_quaternion is None:
        return 0.0

    tw, tx, ty, tz = twist_quaternion
    twist_vector_magnitude = math.sqrt((tx * tx) + (ty * ty) + (tz * tz))
    twist_degrees = 2.0 * math.degrees(math.atan2(twist_vector_magnitude, tw))
    if twist_degrees > 180.0:
        twist_degrees -= 360.0

    if _dot((tx, ty, tz), axis_unit) < 0.0:
        return -abs(twist_degrees)

    return abs(twist_degrees)


def calculate_motion_metrics(
    data: dict[str, Any] | None,
    config: dict[str, Any] | None,
    impact_state: ImpactState,
    impact_cooldown_ms: int = 2000,
) -> dict[str, Any] | None:
    """Calculate derived motion metrics for OSC mapping."""
    if not data or not config:
        return None

    ax = data.get("AX", 0.0)
    ay = data.get("AY", 0.0)
    az = data.get("AZ", 0.0)
    raw_ax = ax
    raw_ay = ay
    raw_az = az
    gx = data.get("GX", 0.0)
    gy = data.get("GY", 0.0)
    gz = data.get("GZ", 0.0)
    yaw = data.get("YAW", 0.0)
    pitch = data.get("PITCH", 0.0)
    roll = data.get("ROLL", 0.0)
    quat_w = data.get("QW")
    quat_x = data.get("QX")
    quat_y = data.get("QY")
    quat_z = data.get("QZ")
    quaternion_now = None

    if all(isinstance(value, (int, float)) for value in (quat_w, quat_x, quat_y, quat_z)):
        quaternion_now = _normalize_quaternion(
            (float(quat_w), float(quat_x), float(quat_y), float(quat_z))
        )

    deadzone = config.get("data_processing", {}).get("deadzone_threshold", 0.05)

    def apply_deadzone(value: float, threshold: float) -> float:
        return 0.0 if abs(value) < threshold else value

    ax = apply_deadzone(ax, deadzone)
    ay = apply_deadzone(ay, deadzone)
    az = apply_deadzone(az, deadzone)
    gx = apply_deadzone(gx, deadzone * 10)
    gy = apply_deadzone(gy, deadzone * 10)
    gz = apply_deadzone(gz, deadzone * 10)

    accel_magnitude = math.sqrt(ax * ax + ay * ay + az * az)
    gyro_magnitude = math.sqrt(gx * gx + gy * gy + gz * gz)

    current_time = get_current_time_ms()
    impact_triggered = False
    pickup_triggered = False
    jerk_gps = 0.0
    jerk_min_gps = 2.0
    edge_mode = "rising"
    rest_g = 0.20
    gyro_active_dps = 120.0
    gyro_rest_dps = 60.0

    impact_cfg_all = config.get(
        "impact_detection",
        config.get("mpu6050_properties", {}).get("impact_detection", {}),
    )
    if impact_cfg_all and impact_cfg_all.get("enabled", False):
        threshold = impact_cfg_all.get("threshold", 8000)
        threshold_g = threshold / 16384.0
        edge_mode = str(impact_cfg_all.get("edge", "rising")).lower()
        rest_g = float(impact_cfg_all.get("rest_g", 0.20))
        min_active_ms = int(impact_cfg_all.get("min_active_ms", 100))
        gyro_active_dps = float(impact_cfg_all.get("gyro_active_dps", 120.0))
        gyro_rest_dps = float(impact_cfg_all.get("gyro_rest_dps", 60.0))
        jerk_min_gps = float(impact_cfg_all.get("jerk_min_gps", 2.0))
        impact_mode = str(impact_cfg_all.get("mode", "composite")).lower()

        baseline_magnitude = 1.0
        deviation_from_baseline = abs(accel_magnitude - baseline_magnitude)

        cooldown_val = impact_cfg_all.get("cooldown_ms", impact_cooldown_ms)
        cooldown_passed = (
            impact_state.last_impact_time == 0
            or current_time - impact_state.last_impact_time > cooldown_val
        )

        if impact_state.last_deviation_time_ms == 0:
            jerk_gps = 0.0
        else:
            dt_s = max(1e-3, (current_time - impact_state.last_deviation_time_ms) / 1000.0)
            jerk_gps = abs(deviation_from_baseline - impact_state.last_deviation_g) / dt_s
        impact_state.last_deviation_g = deviation_from_baseline
        impact_state.last_deviation_time_ms = current_time

        if impact_mode == "jerk":
            if jerk_gps >= jerk_min_gps and cooldown_passed:
                impact_triggered = True
                impact_state.last_impact_time = current_time
        else:
            significant_deviation = deviation_from_baseline > threshold_g
            just_crossed_up = significant_deviation and not impact_state.prev_deviation_above
            if edge_mode in ("rising", "both") and just_crossed_up and cooldown_passed:
                impact_triggered = True
                impact_state.last_impact_time = current_time
            impact_state.prev_deviation_above = significant_deviation

            is_active_now = (
                deviation_from_baseline >= threshold_g
                or gyro_magnitude >= gyro_active_dps
            )
            if not impact_state.motion_active and is_active_now:
                impact_state.motion_active = True
                impact_state.motion_enter_time = current_time

            stop_condition = (
                deviation_from_baseline <= rest_g
                and gyro_magnitude <= gyro_rest_dps
            )
            if impact_state.motion_active and stop_condition:
                active_ms = current_time - impact_state.motion_enter_time
                should_trigger_fall = (
                    edge_mode in ("falling", "both")
                    and active_ms >= min_active_ms
                    and jerk_gps >= jerk_min_gps
                    and cooldown_passed
                )
                if should_trigger_fall:
                    impact_triggered = True
                    impact_state.last_impact_time = current_time
                impact_state.motion_active = False

    # Interaction mode: lights on while object is actively moving,
    # then off again after a short rest hold.
    interaction_cfg = config.get("interaction_mode", {})
    interaction_enabled = interaction_cfg.get("enabled", True)
    baseline_config = interaction_cfg.get("baseline_g")
    if baseline_config is not None:
        interaction_baseline_g = float(baseline_config)
    else:
        baseline_alpha = _clamp(float(interaction_cfg.get("baseline_alpha", 0.02)), 0.0, 1.0)
        baseline_update_gyro_max_dps = float(
            interaction_cfg.get("baseline_update_gyro_max_dps", 3.0)
        )
        if impact_state.interaction_baseline_g is None:
            impact_state.interaction_baseline_g = accel_magnitude
        if (
            not impact_state.interaction_active
            and gyro_magnitude <= baseline_update_gyro_max_dps
        ):
            impact_state.interaction_baseline_g = (
                (1.0 - baseline_alpha) * impact_state.interaction_baseline_g
                + baseline_alpha * accel_magnitude
            )
        interaction_baseline_g = impact_state.interaction_baseline_g

    deviation_from_baseline = abs(accel_magnitude - interaction_baseline_g)
    deviation_from_gravity = abs(accel_magnitude - 1.0)
    accel_on_g = float(interaction_cfg.get("accel_on_g", 0.16))
    accel_off_g = float(interaction_cfg.get("accel_off_g", 0.06))
    gyro_on_dps = float(interaction_cfg.get("gyro_on_dps", 22.0))
    gyro_off_dps = float(interaction_cfg.get("gyro_off_dps", 8.0))
    start_hold_ms = int(interaction_cfg.get("start_hold_ms", 80))
    rest_hold_ms = int(interaction_cfg.get("rest_hold_ms", 250))
    stable_rest_enabled = bool(interaction_cfg.get("stable_rest_enabled", True))
    stable_accel_delta_g = float(interaction_cfg.get("stable_accel_delta_g", 0.006))
    stable_hold_ms = int(interaction_cfg.get("stable_hold_ms", 1800))
    stable_gyro_max_dps = float(interaction_cfg.get("stable_gyro_max_dps", gyro_off_dps))
    active_brightness = _clamp(float(interaction_cfg.get("active_brightness", 1.0)), 0.0, 1.0)
    rest_brightness = _clamp(float(interaction_cfg.get("rest_brightness", 0.0)), 0.0, 1.0)
    stable_rest = False

    if interaction_enabled:
        start_motion = deviation_from_baseline >= accel_on_g or gyro_magnitude >= gyro_on_dps
        stop_motion = deviation_from_baseline <= accel_off_g and gyro_magnitude <= gyro_off_dps

        if impact_state.interaction_active and stable_rest_enabled:
            if gyro_magnitude > stable_gyro_max_dps:
                impact_state.interaction_stable_accel_g = accel_magnitude
                impact_state.interaction_stable_accel_start_time_ms = current_time
            elif impact_state.interaction_stable_accel_g is None:
                impact_state.interaction_stable_accel_g = accel_magnitude
                impact_state.interaction_stable_accel_start_time_ms = current_time
            elif abs(accel_magnitude - impact_state.interaction_stable_accel_g) <= stable_accel_delta_g:
                if impact_state.interaction_stable_accel_start_time_ms == 0:
                    impact_state.interaction_stable_accel_start_time_ms = current_time
                stable_rest = (
                    current_time - impact_state.interaction_stable_accel_start_time_ms
                    >= stable_hold_ms
                )
            else:
                impact_state.interaction_stable_accel_g = accel_magnitude
                impact_state.interaction_stable_accel_start_time_ms = current_time
        else:
            impact_state.interaction_stable_accel_g = None
            impact_state.interaction_stable_accel_start_time_ms = 0

        if not impact_state.interaction_active:
            if start_motion:
                if impact_state.interaction_start_candidate_time_ms == 0:
                    impact_state.interaction_start_candidate_time_ms = current_time
                elif (
                    current_time - impact_state.interaction_start_candidate_time_ms
                    >= start_hold_ms
                ):
                    impact_state.interaction_active = True
                    impact_state.interaction_rest_candidate_time_ms = 0
                    impact_state.interaction_start_candidate_time_ms = 0
                    impact_state.interaction_stable_accel_g = accel_magnitude
                    impact_state.interaction_stable_accel_start_time_ms = current_time
                    pickup_triggered = True
            else:
                impact_state.interaction_start_candidate_time_ms = 0
        elif impact_state.interaction_active:
            impact_state.interaction_start_candidate_time_ms = 0
            if stop_motion or stable_rest:
                if impact_state.interaction_rest_candidate_time_ms == 0:
                    impact_state.interaction_rest_candidate_time_ms = current_time
                elif current_time - impact_state.interaction_rest_candidate_time_ms >= rest_hold_ms:
                    impact_state.interaction_active = False
                    impact_state.interaction_stable_accel_g = None
                    impact_state.interaction_stable_accel_start_time_ms = 0
            else:
                impact_state.interaction_rest_candidate_time_ms = 0
    else:
        impact_state.interaction_active = True
        impact_state.interaction_start_candidate_time_ms = 0
        impact_state.interaction_rest_candidate_time_ms = 0
        impact_state.interaction_stable_accel_g = None
        impact_state.interaction_stable_accel_start_time_ms = 0

    interaction_gate = (
        active_brightness if impact_state.interaction_active else rest_brightness
    )
    accel_activity = _scale_to_unit(deviation_from_baseline, accel_off_g, accel_on_g)
    gyro_activity = _scale_to_unit(gyro_magnitude, gyro_off_dps, gyro_on_dps)
    interaction_activity = max(accel_activity, gyro_activity)
    if not impact_state.interaction_active:
        interaction_activity = 0.0

    synth_cfg = config.get("synth_settings", {})
    tilt_cfg = synth_cfg.get("tilt_mapping", {}) if isinstance(synth_cfg, dict) else {}
    tilt_mode = tilt_cfg.get("mode", "raw_orientation")
    synth_yaw_norm = (float(yaw) % 360.0) / 360.0
    synth_pitch_norm = _clamp(pitch / 90.0, -1.0, 1.0)
    synth_roll_norm = _clamp(roll / 90.0, -1.0, 1.0)
    synth_pitch_deg = pitch
    synth_roll_deg = roll
    synth_horizontal_strength = 0.0
    synth_yaw_active = False
    synth_axial_gyro_dps = 0.0

    if tilt_mode == "calibrated_gravity":
        gravity_now = _normalize_vector(raw_ax, raw_ay, raw_az)
        pitch_axis_cfg = tilt_cfg.get("pitch_axis_unit")
        roll_axis_cfg = tilt_cfg.get("roll_axis_unit")
        rest_filter_alpha = _clamp(float(tilt_cfg.get("filter_alpha_rest", 0.18)), 0.0, 1.0)
        active_filter_alpha = _clamp(float(tilt_cfg.get("filter_alpha_active", 0.65)), 0.0, 1.0)
        filter_alpha = active_filter_alpha if impact_state.interaction_active else rest_filter_alpha
        pitch_sign = -1.0 if tilt_cfg.get("pitch_invert", False) else 1.0
        roll_sign = -1.0 if tilt_cfg.get("roll_invert", False) else 1.0
        yaw_mode = str(tilt_cfg.get("yaw_mode", "horizontal_axis_heading")).lower()
        yaw_horizontal_threshold = _clamp(
            float(tilt_cfg.get("yaw_horizontal_threshold", 0.6)), 0.0, 0.999
        )
        yaw_filter_alpha = _clamp(float(tilt_cfg.get("yaw_filter_alpha", 0.28)), 0.0, 1.0)
        yaw_axial_suppress_threshold = max(
            float(tilt_cfg.get("yaw_axial_suppress_threshold", 2.5)), 0.0
        )
        roll_mode = str(tilt_cfg.get("roll_mode", "quaternion_body")).lower()
        roll_full_scale_deg = max(float(tilt_cfg.get("roll_full_scale_deg", 180.0)), 1.0)
        roll_gyro_deadzone_dps = max(float(tilt_cfg.get("roll_gyro_deadzone_dps", 2.5)), 0.0)
        roll_leak_per_second = _clamp(
            float(tilt_cfg.get("roll_leak_per_second", 0.08)), 0.0, 1.0
        )
        last_orientation_time_ms = impact_state.synth_last_orientation_update_ms
        dt_s = 0.0

        if last_orientation_time_ms > 0:
            dt_s = _clamp((current_time - last_orientation_time_ms) / 1000.0, 0.0, 0.25)

        impact_state.synth_last_orientation_update_ms = current_time

        if (
            gravity_now is not None
            and isinstance(pitch_axis_cfg, list)
            and len(pitch_axis_cfg) == 3
            and isinstance(roll_axis_cfg, list)
            and len(roll_axis_cfg) == 3
        ):
            if impact_state.synth_filtered_gravity is None:
                impact_state.synth_filtered_gravity = gravity_now
            else:
                prev = impact_state.synth_filtered_gravity
                mixed = (
                    ((1.0 - filter_alpha) * prev[0]) + (filter_alpha * gravity_now[0]),
                    ((1.0 - filter_alpha) * prev[1]) + (filter_alpha * gravity_now[1]),
                    ((1.0 - filter_alpha) * prev[2]) + (filter_alpha * gravity_now[2]),
                )
                impact_state.synth_filtered_gravity = _normalize_vector(*mixed) or gravity_now

            filtered = impact_state.synth_filtered_gravity
            long_axis = _normalize_vector(*[float(v) for v in pitch_axis_cfg])
            pitch_axis = _normalize_vector(*[float(v) for v in pitch_axis_cfg])
            roll_axis = _normalize_vector(*[float(v) for v in roll_axis_cfg])
            if filtered and long_axis and pitch_axis and roll_axis:
                long_axis_world = None
                roll_axis_world = None
                synth_pitch_norm = _clamp(pitch_sign * _dot(filtered, pitch_axis), -1.0, 1.0)
                synth_pitch_deg = math.degrees(math.asin(synth_pitch_norm))
                horizontal_strength = math.sqrt(
                    max(0.0, 1.0 - (_clamp(_dot(filtered, long_axis), -1.0, 1.0) ** 2))
                )
                synth_horizontal_strength = horizontal_strength

                axial_gyro_dps = roll_sign * _dot((gx, gy, gz), long_axis)
                if abs(axial_gyro_dps) < roll_gyro_deadzone_dps:
                    axial_gyro_dps = 0.0
                synth_axial_gyro_dps = axial_gyro_dps

                if yaw_mode in ("horizontal_axis_heading", "horizontal_dmp"):
                    previous_horizontal_yaw = impact_state.synth_horizontal_yaw_norm
                    synth_yaw_norm = previous_horizontal_yaw if previous_horizontal_yaw is not None else 0.0
                    yaw_update_allowed = horizontal_strength >= yaw_horizontal_threshold
                    raw_yaw_norm = None
                    suppress_yaw_from_axial_roll = True

                    if quaternion_now is not None:
                        long_axis_world = _rotate_vector_by_quaternion(quaternion_now, long_axis)
                        roll_axis_world = _rotate_vector_by_quaternion(quaternion_now, roll_axis)

                        synth_horizontal_strength = math.sqrt(
                            max(0.0, 1.0 - (_clamp(long_axis_world[2], -1.0, 1.0) ** 2))
                        )
                        yaw_update_allowed = synth_horizontal_strength >= yaw_horizontal_threshold
                        heading_world = _project_onto_plane(long_axis_world, (0.0, 0.0, 1.0))
                        if heading_world is not None:
                            raw_yaw_norm = math.atan2(heading_world[1], heading_world[0]) / (2.0 * math.pi)
                            if raw_yaw_norm < 0.0:
                                raw_yaw_norm += 1.0
                            # Quaternion-projected heading is already insensitive to pure long-axis roll,
                            # so freezing yaw on fast motion here only makes the live response lag.
                            suppress_yaw_from_axial_roll = False
                    else:
                        roll_axis_world = None
                        if yaw_update_allowed:
                            raw_yaw_norm = (float(yaw) % 360.0) / 360.0

                    if (
                        yaw_update_allowed
                        and (raw_yaw_norm is not None)
                        and (
                            (suppress_yaw_from_axial_roll is False)
                            or (abs(axial_gyro_dps) < yaw_axial_suppress_threshold)
                        )
                    ):
                        synth_yaw_active = True
                        if previous_horizontal_yaw is None:
                            synth_yaw_norm = raw_yaw_norm
                        else:
                            synth_yaw_norm = (
                                previous_horizontal_yaw
                                + (
                                    _wrap_unit_delta(raw_yaw_norm, previous_horizontal_yaw)
                                    * yaw_filter_alpha
                                )
                            ) % 1.0

                        impact_state.synth_horizontal_yaw_norm = synth_yaw_norm

                if roll_mode == "quaternion_body" and (quaternion_now is not None):
                    if impact_state.synth_roll_reference_quaternion is None:
                        impact_state.synth_roll_reference_quaternion = quaternion_now

                    measured_roll_deg = _quaternion_relative_twist_degrees(
                        impact_state.synth_roll_reference_quaternion,
                        quaternion_now,
                        long_axis,
                    )
                    if measured_roll_deg is not None:
                        measured_roll_deg = roll_sign * measured_roll_deg
                        previous_roll_measurement_deg = impact_state.synth_last_roll_measurement_deg

                        if previous_roll_measurement_deg is None:
                            synth_roll_deg = measured_roll_deg
                        else:
                            synth_roll_deg = impact_state.synth_axis_roll_deg + _wrap_signed_degrees(
                                measured_roll_deg - previous_roll_measurement_deg
                            )

                        impact_state.synth_last_roll_measurement_deg = measured_roll_deg
                        impact_state.synth_axis_roll_deg = synth_roll_deg
                        synth_roll_norm = _clamp(synth_roll_deg / roll_full_scale_deg, -1.0, 1.0)
                elif roll_mode == "axial_gyro":
                    impact_state.synth_roll_reference_quaternion = None
                    impact_state.synth_last_roll_measurement_deg = None
                    roll_deg = impact_state.synth_axis_roll_deg
                    if dt_s > 0.0:
                        roll_deg += axial_gyro_dps * dt_s
                        if axial_gyro_dps == 0.0 and roll_leak_per_second > 0.0:
                            roll_deg *= max(0.0, 1.0 - (roll_leak_per_second * dt_s))

                    roll_deg = _wrap_signed_degrees(roll_deg)
                    impact_state.synth_axis_roll_deg = roll_deg
                    synth_roll_deg = roll_deg
                    synth_roll_norm = _clamp(roll_deg / roll_full_scale_deg, -1.0, 1.0)
                else:
                    impact_state.synth_roll_reference_quaternion = None
                    impact_state.synth_last_roll_measurement_deg = None
                    synth_roll_norm = _clamp(roll_sign * _dot(filtered, roll_axis), -1.0, 1.0)
                    synth_roll_deg = math.degrees(math.asin(synth_roll_norm))

    return {
        "accelerometer_x": ax,
        "accelerometer_y": ay,
        "accelerometer_z": az,
        "gyroscope_x": gx,
        "gyroscope_y": gy,
        "gyroscope_z": gz,
        "orientation_yaw": yaw,
        "orientation_pitch": pitch,
        "orientation_roll": roll,
        "accel_magnitude": accel_magnitude,
        "gyro_magnitude": gyro_magnitude,
        "impact_triggered": impact_triggered,
        "pickup_triggered": pickup_triggered,
        "deviation_from_baseline_g": deviation_from_gravity,
        "interaction_deviation_g": deviation_from_baseline,
        "threshold_g": (
            config.get("impact_detection", {}).get("threshold", 8000) / 16384.0
        ),
        "time_since_last_impact_ms": (
            current_time - impact_state.last_impact_time
            if impact_state.last_impact_time
            else None
        ),
        "jerk_gps": jerk_gps,
        "jerk_threshold_gps": jerk_min_gps,
        "edge_mode": edge_mode,
        "rest_g": rest_g,
        "motion_active_state": impact_state.motion_active,
        "gyro_active_dps": gyro_active_dps,
        "gyro_rest_dps": gyro_rest_dps,
        "interaction_active": impact_state.interaction_active,
        "interaction_gate": interaction_gate,
        "interaction_activity": interaction_activity,
        "interaction_accel_activity": accel_activity,
        "interaction_gyro_activity": gyro_activity,
        "interaction_accel_on_g": accel_on_g,
        "interaction_accel_off_g": accel_off_g,
        "interaction_gyro_on_dps": gyro_on_dps,
        "interaction_gyro_off_dps": gyro_off_dps,
        "interaction_start_hold_ms": start_hold_ms,
        "interaction_rest_hold_ms": rest_hold_ms,
        "interaction_baseline_g": interaction_baseline_g,
        "interaction_stable_rest": stable_rest,
        "interaction_stable_accel_g": impact_state.interaction_stable_accel_g,
        "interaction_stable_accel_delta_g": (
            abs(accel_magnitude - impact_state.interaction_stable_accel_g)
            if impact_state.interaction_stable_accel_g is not None
            else 0.0
        ),
        "interaction_stable_hold_ms": stable_hold_ms,
        "synth_yaw_norm": synth_yaw_norm,
        "synth_pitch_norm": synth_pitch_norm,
        "synth_roll_norm": synth_roll_norm,
        "synth_pitch_deg": synth_pitch_deg,
        "synth_roll_deg": synth_roll_deg,
        "synth_horizontal_strength": synth_horizontal_strength,
        "synth_yaw_active": synth_yaw_active,
        "synth_axial_gyro_dps": synth_axial_gyro_dps,
        "active_duration_ms": (
            current_time - impact_state.motion_enter_time
            if impact_state.motion_active
            else 0
        ),
    }
