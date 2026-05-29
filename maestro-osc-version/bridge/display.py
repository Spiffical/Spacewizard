"""Terminal display helpers for test and impact-debug modes."""

from __future__ import annotations

import time
from typing import Any

from .metrics import ImpactState, get_current_time_ms
from .osc import preview_osc_messages


_compact_test_display_initialized = False
_compact_test_display_width = 0


def display_compact_test_data(
    data: dict[str, Any],
    metrics: dict[str, Any],
) -> None:
    """Display a lightweight single-line status for responsive test mode."""
    global _compact_test_display_initialized, _compact_test_display_width

    if not _compact_test_display_initialized:
        print("🔬 SpaceWizard Motion Data Test Mode (compact)")
        print("Press Ctrl+C to exit | use --verbose-test for the full table")
        _compact_test_display_initialized = True

    interaction_state = "ACTIVE" if metrics.get("interaction_active", False) else "REST"
    impact_state = "IMPACT" if metrics.get("impact_triggered", False) else "----"
    status_line = (
        f"{time.strftime('%H:%M:%S')} "
        f"AX {data.get('AX', 0.0):+0.3f} "
        f"AY {data.get('AY', 0.0):+0.3f} "
        f"AZ {data.get('AZ', 0.0):+0.3f}  "
        f"GX {data.get('GX', 0.0):+0.1f} "
        f"GY {data.get('GY', 0.0):+0.1f} "
        f"GZ {data.get('GZ', 0.0):+0.1f}  "
        f"Y {data.get('YAW', 0.0):+6.1f} "
        f"P {data.get('PITCH', 0.0):+6.1f} "
        f"R {data.get('ROLL', 0.0):+6.1f}  "
        f"SP {metrics.get('synth_pitch_deg', 0.0):+6.1f} "
        f"SR {metrics.get('synth_roll_deg', 0.0):+6.1f}  "
        f"Gate {metrics.get('interaction_gate', 0.0):0.2f} "
        f"Act {metrics.get('interaction_activity', 0.0):0.2f} "
        f"Dev {metrics.get('interaction_deviation_g', 0.0):0.3f}  "
        f"{interaction_state:<6} {impact_state}"
    )
    _compact_test_display_width = max(_compact_test_display_width, len(status_line))
    print(f"\r{status_line.ljust(_compact_test_display_width)}", end="", flush=True)


def display_test_data(
    data: dict[str, Any],
    metrics: dict[str, Any],
    config: dict[str, Any],
) -> None:
    """Display motion data in a readable table for test mode."""
    print("\033[2J\033[H", end="")

    print("🔬 SpaceWizard Motion Data Test Mode")
    print("=" * 70)
    print(f"Timestamp: {time.strftime('%H:%M:%S')}")
    print()

    print("📊 RAW SENSOR DATA:")
    print("-" * 50)
    print(f"{'Accelerometer (g):':>20} | {data.get('AX', 0):>8.3f} | {data.get('AY', 0):>8.3f} | {data.get('AZ', 0):>8.3f}")
    print(f"{'Gyroscope (°/s):':>20} | {data.get('GX', 0):>8.1f} | {data.get('GY', 0):>8.1f} | {data.get('GZ', 0):>8.1f}")
    print()

    print("🧭 ORIENTATION:")
    print("-" * 50)
    print(f"{'Yaw (degrees):':>20} | {data.get('YAW', 0):>8.1f}°")
    print(f"{'Pitch (degrees):':>20} | {data.get('PITCH', 0):>8.1f}°")
    print(f"{'Roll (degrees):':>20} | {data.get('ROLL', 0):>8.1f}°")
    print()

    print("⚡ CALCULATED METRICS:")
    print("-" * 50)
    print(f"{'Accel Magnitude (g):':>20} | {metrics.get('accel_magnitude', 0):>8.3f}")
    print(f"{'Gyro Magnitude (°/s):':>20} | {metrics.get('gyro_magnitude', 0):>8.1f}")
    print()

    impact_status = "🚨 IMPACT DETECTED!" if metrics.get("impact_triggered", False) else "✅ No impact"
    print(f"{'Impact Status:':>20} | {impact_status}")
    interaction_status = "🟢 Active" if metrics.get("interaction_active", False) else "⚫ Resting"
    print(f"{'Interaction State:':>20} | {interaction_status}")
    print(f"{'Interaction Gate:':>20} | {metrics.get('interaction_gate', 0.0):>8.2f}")
    print(f"{'Interaction Activity:':>20} | {metrics.get('interaction_activity', 0.0):>8.2f}")
    print(f"{'Interaction Dev (g):':>20} | {metrics.get('interaction_deviation_g', 0.0):>8.3f}")
    print(f"{'Synth Pitch (deg):':>20} | {metrics.get('synth_pitch_deg', 0.0):>8.1f}°")
    print(f"{'Synth Roll (deg):':>20} | {metrics.get('synth_roll_deg', 0.0):>8.1f}°")
    print(f"{'Synth Pitch Norm:':>20} | {metrics.get('synth_pitch_norm', 0.0):>8.3f}")
    print(f"{'Synth Roll Norm:':>20} | {metrics.get('synth_roll_norm', 0.0):>8.3f}")
    print()

    print("🎛️  OSC MESSAGES (Preview):")
    print("-" * 50)
    messages = preview_osc_messages(metrics, config)
    if messages:
        for msg in messages[:5]:
            print(f"  → {msg}")
        if len(messages) > 5:
            print(f"  ... and {len(messages) - 5} more messages")
    else:
        print("  No OSC messages configured")

    print()
    print("=" * 70)
    print("💡 Tip: Move your SpaceWizard device to see real-time motion data!")
    print("   Press Ctrl+C to exit test mode")


def display_impact_test_data(
    data: dict[str, Any],
    metrics: dict[str, Any],
    config: dict[str, Any],
    impact_state: ImpactState,
    session_max_deviation_g: float,
) -> float:
    """Display detailed impact detection analysis and return updated session max."""
    print("\033[2J\033[H", end="")

    print("🎯 SpaceWizard Impact Detection Test Mode")
    print("=" * 80)
    print(f"Timestamp: {time.strftime('%H:%M:%S')}")
    print()

    ax = data.get("AX", 0.0)
    ay = data.get("AY", 0.0)
    az = data.get("AZ", 0.0)

    print("📊 ACCELEROMETER ANALYSIS:")
    print("-" * 60)
    is_raw_counts = max(abs(ax), abs(ay), abs(az)) > 4.0
    ax_g = (ax / 16384.0) if is_raw_counts else ax
    ay_g = (ay / 16384.0) if is_raw_counts else ay
    az_g = (az / 16384.0) if is_raw_counts else az
    print(f"{'Raw Values:':>15} | X: {ax:>8.3f} | Y: {ay:>8.3f} | Z: {az:>8.3f}")
    print(f"{'As g-force:':>15} | X: {ax_g:>8.3f} | Y: {ay_g:>8.3f} | Z: {az_g:>8.3f}")

    accel_magnitude = metrics.get("accel_magnitude", 0.0)
    baseline_magnitude = 1.0
    deviation_from_baseline = abs(accel_magnitude - baseline_magnitude)

    print(f"{'Magnitude:':>15} | Total: {accel_magnitude:>8.3f}g")
    print(f"{'Baseline (1g):':>15} | Expected: {baseline_magnitude:>8.3f}g")
    print(f"{'Deviation:':>15} | Difference: {deviation_from_baseline:>8.3f}g")
    print()

    print("🎯 IMPACT DETECTION ANALYSIS:")
    print("-" * 60)
    impact_cfg_all = config.get(
        "impact_detection",
        config.get("mpu6050_properties", {}).get("impact_detection", {}),
    )
    threshold = impact_cfg_all.get("threshold", 8000)
    threshold_g = threshold / 16384.0

    current_time = get_current_time_ms()
    time_since_last_impact = (
        current_time - impact_state.last_impact_time
        if impact_state.last_impact_time
        else None
    )
    cooldown_ms = impact_cfg_all.get("cooldown_ms", 2000)
    cooldown_remaining = (
        cooldown_ms - time_since_last_impact
        if time_since_last_impact is not None
        else None
    )
    if cooldown_remaining is not None and cooldown_remaining < 0:
        cooldown_remaining = 0

    print(f"{'Threshold:':>15} | Raw: {threshold:>6d} | In g: {threshold_g:>6.3f}g")
    if cooldown_remaining is not None:
        print(f"{'Cooldown:':>15} | {cooldown_remaining:>6d}ms remaining")
    else:
        print(f"{'Cooldown:':>15} | {'N/A':>6}")
    if time_since_last_impact is not None:
        print(f"{'Last Impact:':>15} | {time_since_last_impact:>6d}ms ago")
    else:
        print(f"{'Last Impact:':>15} | {'never':>6}")
    print()

    print("🔍 DECISION LOGIC:")
    print("-" * 60)

    significant_deviation = deviation_from_baseline > threshold_g
    cooldown_passed = (time_since_last_impact is None) or (time_since_last_impact > cooldown_ms)
    impact_triggered = significant_deviation and cooldown_passed

    print(
        f"{'Deviation > Threshold:':>25} | {str(significant_deviation):>5} "
        f"(Deviation: {deviation_from_baseline:.3f}g > {threshold_g:.3f}g)"
    )
    if time_since_last_impact is not None:
        print(
            f"{'Cooldown Passed:':>25} | {str(cooldown_passed):>5} "
            f"(Time: {time_since_last_impact}ms > {cooldown_ms}ms)"
        )
    else:
        print(f"{'Cooldown Passed:':>25} | {str(cooldown_passed):>5} (No prior impacts)")
    print(f"{'Impact Triggered:':>25} | {'🚨 YES' if impact_triggered else '✅ NO':>5}")
    print()

    print("🎛️ INTERACTION MODE:")
    print("-" * 60)
    interaction_state = "ACTIVE" if metrics.get("interaction_active", False) else "RESTING"
    print(f"{'State:':>25} | {interaction_state:>8}")
    print(f"{'Gate (0-1):':>25} | {metrics.get('interaction_gate', 0.0):>8.2f}")
    print(f"{'Activity (0-1):':>25} | {metrics.get('interaction_activity', 0.0):>8.2f}")
    interaction_deviation = metrics.get("interaction_deviation_g", 0.0)
    print(
        f"{'Accel dev vs thresholds:':>25} | "
        f"{interaction_deviation:>5.3f}g "
        f"(on {metrics.get('interaction_accel_on_g', 0.0):.3f} / "
        f"off {metrics.get('interaction_accel_off_g', 0.0):.3f})"
    )
    print(
        f"{'Gyro mag vs thresholds:':>25} | "
        f"{metrics.get('gyro_magnitude', 0.0):>5.1f}°/s "
        f"(on {metrics.get('interaction_gyro_on_dps', 0.0):.1f} / "
        f"off {metrics.get('interaction_gyro_off_dps', 0.0):.1f})"
    )
    print()

    if deviation_from_baseline > session_max_deviation_g:
        session_max_deviation_g = deviation_from_baseline

    recommended_threshold_g = session_max_deviation_g * 1.10
    recommended_threshold_raw = int(recommended_threshold_g * 16384)

    print()
    print("-" * 80)
    print(
        f"Session Max Deviation: {session_max_deviation_g:.3f}g | "
        f"Suggested threshold: {recommended_threshold_g:.3f}g (~{recommended_threshold_raw} raw)"
    )
    summary_parts = [
        f"ts_ms={current_time}",
        f"ax_g={ax_g:.3f}",
        f"ay_g={ay_g:.3f}",
        f"az_g={az_g:.3f}",
        f"mag_g={accel_magnitude:.3f}",
        f"dev_g={deviation_from_baseline:.3f}",
        f"thr_g={threshold_g:.3f}",
        f"thr_raw={threshold}",
        f"since_ms={(time_since_last_impact if time_since_last_impact is not None else -1)}",
        f"rem_ms={(cooldown_remaining if cooldown_remaining is not None else -1)}",
        f"cooldown_ms={cooldown_ms}",
        f"sig={int(significant_deviation)}",
        f"cool={int(cooldown_passed)}",
        f"trig={int(impact_triggered)}",
    ]
    print("IMPACT_SUMMARY " + " ".join(summary_parts))

    print()
    print("=" * 80)
    print("💡 DEBUGGING TIPS:")
    print("   • Green checkmarks = condition met")
    print("   • Red X marks = condition not met")
    print("   • Adjust threshold in config.json if too sensitive/insensitive")
    print("   • Check if sensor is perfectly level (affects baseline)")
    print("   • Press Ctrl+C to exit impact test mode")

    return session_max_deviation_g
