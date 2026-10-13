"""OSC mapping resolution and sending helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp value between min and max."""
    return max(min_val, min(value, max_val))


def apply_mapping_transform(value: float, mapping: dict[str, Any]) -> float:
    """Apply scaling, inversion, and clamping based on mapping configuration."""
    scale_min = mapping.get("scale_min", 0.0)
    scale_max = mapping.get("scale_max", 1.0)
    invert = mapping.get("invert", False)

    if scale_max != scale_min:
        scaled = (value - scale_min) / (scale_max - scale_min)
    else:
        scaled = 0.0 if value < scale_min else 1.0

    scaled = clamp(scaled, 0.0, 1.0)
    if invert:
        scaled = 1.0 - scaled
    return scaled


@dataclass
class ResolvedOscMessage:
    address: str
    value: Any
    source: str | None
    mapping_type: str
    preview: str


def _resolve_flat_mapping_messages(
    metrics: dict[str, Any],
    config: dict[str, Any],
) -> list[ResolvedOscMessage]:
    messages: list[ResolvedOscMessage] = []
    flat_mappings = config.get("osc_mappings", [])
    if not isinstance(flat_mappings, list) or not flat_mappings:
        return messages

    for mapping in flat_mappings:
        if mapping.get("enabled", True) is False:
            continue

        osc_address = mapping.get("osc_address")
        source = mapping.get("source")
        if not osc_address or not source:
            continue

        invert = mapping.get("invert", False)
        is_threshold = mapping.get("threshold", False)

        abs_mode = False
        const_mode = False
        resolved_source = source
        if isinstance(resolved_source, str):
            if resolved_source.startswith("abs:"):
                abs_mode = True
                resolved_source = resolved_source.split(":", 1)[1]
            elif resolved_source.startswith("const:"):
                const_mode = True

        if const_mode:
            try:
                value = float(resolved_source.split(":", 1)[1])
            except ValueError:
                continue
        else:
            value = metrics.get(resolved_source)
        if value is None:
            continue
        if abs_mode and isinstance(value, (int, float)):
            value = abs(value)

        if is_threshold:
            bool_value = bool(value)
            messages.append(
                ResolvedOscMessage(
                    address=osc_address,
                    value=bool_value,
                    source=resolved_source,
                    mapping_type="flat-threshold",
                    preview=f"{osc_address}={'True' if bool_value else 'False'} (flat)",
                )
            )
            continue

        scaled_value = value
        if isinstance(value, (int, float)):
            scale_min = mapping.get("scale_min", 0.0)
            scale_max = mapping.get("scale_max", 1.0)
            if scale_max != scale_min:
                scaled_value = (value - scale_min) / (scale_max - scale_min)
            else:
                scaled_value = 0.0 if value < scale_min else 1.0
            scaled_value = clamp(scaled_value, 0.0, 1.0)
            if invert:
                scaled_value = 1.0 - scaled_value

        preview_value = f"{scaled_value:.3f}" if isinstance(scaled_value, float) else str(scaled_value)
        messages.append(
            ResolvedOscMessage(
                address=osc_address,
                value=scaled_value,
                source=resolved_source,
                mapping_type="flat",
                preview=f"{osc_address}={preview_value} (flat)",
            )
        )

    return messages


def _resolve_legacy_mapping_messages(
    metrics: dict[str, Any],
    config: dict[str, Any],
) -> list[ResolvedOscMessage]:
    messages: list[ResolvedOscMessage] = []
    properties = config.get("mpu6050_properties", {})
    for prop_name, prop_config in properties.items():
        if not prop_config.get("enabled", False):
            continue
        value = metrics.get(prop_name.lower())
        if value is None:
            continue

        mappings = prop_config.get("maestro_mapping", [])
        for mapping in mappings:
            osc_address = mapping.get("osc_address")
            if not osc_address:
                continue

            scaled_value = apply_mapping_transform(value, mapping)
            if mapping.get("threshold", False):
                threshold_min = mapping.get("scale_min", 0)
                threshold_max = mapping.get("scale_max", 1)
                bool_value = threshold_min <= scaled_value <= threshold_max
                messages.append(
                    ResolvedOscMessage(
                        address=osc_address,
                        value=bool_value,
                        source=prop_name,
                        mapping_type="property-threshold",
                        preview=f"{osc_address}={'True' if bool_value else 'False'} (threshold)",
                    )
                )
            else:
                messages.append(
                    ResolvedOscMessage(
                        address=osc_address,
                        value=scaled_value,
                        source=prop_name,
                        mapping_type="property",
                        preview=f"{osc_address}={scaled_value:.3f}",
                    )
                )
    return messages


def resolve_osc_messages(
    metrics: dict[str, Any],
    config: dict[str, Any],
) -> list[ResolvedOscMessage]:
    """Resolve all enabled OSC messages for one metric frame."""
    flat_messages = _resolve_flat_mapping_messages(metrics, config)
    if flat_messages:
        return flat_messages
    return _resolve_legacy_mapping_messages(metrics, config)


def preview_osc_messages(metrics: dict[str, Any], config: dict[str, Any]) -> list[str]:
    """Return preview strings for enabled OSC mappings."""
    return [msg.preview for msg in resolve_osc_messages(metrics, config)]


def send_osc_messages(
    metrics: dict[str, Any] | None,
    config: dict[str, Any] | None,
    osc_client: Any,
) -> None:
    """Send OSC messages to MaestroDMX based on config and motion metrics."""
    if not metrics or not config or not osc_client:
        return

    debug_enabled = config.get("data_processing", {}).get("enable_debug_output", False)
    sent_previews: list[str] = []
    error_count = 0
    first_error: dict[str, Any] | None = None

    for msg in resolve_osc_messages(metrics, config):
        try:
            if osc_client.send_message(msg.address, msg.value) is not False:
                sent_previews.append(msg.preview)
        except Exception as exc:  # pragma: no cover - exercised with live networking
            error_count += 1
            if first_error is None:
                first_error = {
                    "address": msg.address,
                    "value": msg.value,
                    "source": msg.source,
                    "mapping_type": msg.mapping_type,
                    "error": exc,
                }

    if debug_enabled and sent_previews:
        suffix = "..." if len(sent_previews) > 3 else ""
        print(f"  → OSC: {', '.join(sent_previews[:3])}{suffix}")

    if error_count and first_error:
        err = first_error["error"]
        if debug_enabled:
            import errno as _errno

            osc_host = getattr(osc_client, "_address", None)
            osc_port = getattr(osc_client, "_port", None)
            target = f"{osc_host}:{osc_port}" if osc_host and osc_port else "unknown"

            err_no = getattr(err, "errno", None)
            err_code = _errno.errorcode.get(err_no, "UNKNOWN") if err_no is not None else None
            err_meta = f" errno={err_no}({err_code})" if err_no is not None else ""

            value_repr = first_error["value"]
            if isinstance(value_repr, float):
                value_repr = f"{value_repr:.4f}"

            print(
                "OSC send failed "
                f"(count={error_count}, target={target}, "
                f"addr={first_error['address']}, "
                f"value={value_repr}, "
                f"source={first_error['source']}, "
                f"mapping={first_error['mapping_type']}, "
                f"error={err}{err_meta})"
            )
        else:
            print(f"Error sending OSC message: {err}")


def send_synth_control_messages(
    metrics: dict[str, Any] | None,
    synth_client: Any,
) -> None:
    """Send a compact OSC control packet for the SuperCollider synth patch."""
    if not metrics or not synth_client:
        return

    gate, activity, pitch_norm, roll_norm, spin_norm, yaw_norm = _build_synth_control_payload(metrics)

    synth_client.send_message(
        "/spacewizard/auduino",
        [gate, activity, pitch_norm, roll_norm, spin_norm, yaw_norm,
         1.0 if metrics.get("input_valid", True) else 0.0,
         # Song mode only: is the song playing, and its motion energy (0..1, drives speed,
         # pitch and volume). Both are 0 when input is invalid.
         1.0 if (metrics.get("song_gate_active", False) and metrics.get("input_valid", True)) else 0.0,
         clamp(float(metrics.get("song_energy", 0.0)), 0.0, 1.0) if metrics.get("input_valid", True) else 0.0],
    )


def send_synth_impact_messages(
    metrics: dict[str, Any] | None,
    synth_client: Any,
) -> None:
    """Send a one-shot impact trigger to the local SuperCollider synth patch."""
    if not metrics or not synth_client:
        return
    if not bool(metrics.get("impact_triggered", False)):
        return

    synth_client.send_message(
        "/spacewizard/impact",
        [
            1.0,
            clamp(float(metrics.get("jerk_gps", 0.0)), 0.0, 200.0),
            clamp(float(metrics.get("interaction_deviation_g", 0.0)), 0.0, 2.0),
            clamp(float(metrics.get("interaction_activity", 0.0)), 0.0, 1.0),
        ],
    )


def _build_synth_control_payload(metrics: dict[str, Any]) -> tuple[float, float, float, float, float, float]:
    """Build the shared synth/status payload from one motion metrics frame."""
    gate = 1.0 if bool(metrics.get("interaction_active", False)) else 0.0
    activity = clamp(float(metrics.get("interaction_activity", 0.0)), 0.0, 1.0)
    pitch_norm = clamp(float(metrics.get("synth_pitch_norm", 0.0)), -1.0, 1.0)
    roll_norm = clamp(float(metrics.get("synth_roll_norm", 0.0)), -1.0, 1.0)
    spin_norm = clamp(float(metrics.get("interaction_gyro_activity", 0.0)), 0.0, 1.0)
    yaw_norm = clamp(float(metrics.get("synth_yaw_norm", 0.0)), 0.0, 1.0)

    return gate, activity, pitch_norm, roll_norm, spin_norm, yaw_norm


def send_synth_status_messages(
    metrics: dict[str, Any] | None,
    synth_client: Any,
) -> None:
    """Send a lower-rate status packet for the SuperCollider live status GUI."""
    if not metrics or not synth_client:
        return

    gate, activity, pitch_norm, roll_norm, spin_norm, yaw_norm = _build_synth_control_payload(metrics)

    synth_client.send_message(
        "/spacewizard/live_status",
        [gate, activity, pitch_norm, roll_norm, spin_norm, yaw_norm],
    )


def send_safe_state(config: dict[str, Any], osc_client: Any, synth_client: Any) -> None:
    """Explicitly extinguish outputs, independent of configured resting brightness."""
    if osc_client:
        values = {"/global/brightness": 0.0, "/live/1/brightness": 0.0}
        for mapping in config.get("osc_mappings", []):
            address = mapping.get("osc_address", "")
            if mapping.get("enabled", True) and address:
                if address.endswith("/brightness"):
                    values[address] = 0.0
                elif mapping.get("threshold", False):
                    values[address] = False
        for address, value in values.items():
            osc_client.send_message(address, value)
    if synth_client:
        safe = {"interaction_active": False, "input_valid": False}
        send_synth_control_messages(safe, synth_client)
        send_synth_status_messages(safe, synth_client)
