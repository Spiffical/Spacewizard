#!/usr/bin/env python3
"""Guided capture for diagnosing SpaceWizard orientation-mapping behavior."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import subprocess
import time

import serial

from bridge.config import load_config
from bridge.metrics import ImpactState, calculate_motion_metrics
from bridge.serial_input import find_arduino_port, parse_serial_line


@dataclass(frozen=True)
class Stage:
    name: str
    instruction: str
    prep_s: float
    duration_s: float


@dataclass(frozen=True)
class TraceProtocol:
    name: str
    description: str
    output_prefix: str
    stages: tuple[Stage, ...]
    key_diagnostics: tuple[tuple[str, str], ...]


TRACE_PROTOCOLS = {
    "axis-mapping": TraceProtocol(
        name="axis-mapping",
        description="Original horizontal yaw vs long-axis roll diagnostic.",
        output_prefix="axis_mapping_trace",
        stages=(
            Stage(
                "HORIZONTAL_HOLD",
                "Hold the candle horizontal in front of you and keep it still.",
                4.0,
                4.0,
            ),
            Stage(
                "HORIZONTAL_AXIS_ROLL",
                "Keep the candle horizontal at the same yaw and twist it only along its long axis.",
                3.0,
                8.0,
            ),
            Stage(
                "HORIZONTAL_RESET",
                "Stop twisting and hold the candle horizontal and still again.",
                2.0,
                3.0,
            ),
            Stage(
                "RAISE_TO_PITCH_90",
                "Raise the pitch toward +90 degrees while keeping roughly the same yaw.",
                3.0,
                5.0,
            ),
            Stage(
                "PITCH_90_AXIS_ROLL",
                "At about +90 pitch, twist the candle only along its long axis again.",
                3.0,
                8.0,
            ),
            Stage(
                "RETURN_TO_HORIZONTAL",
                "Lower the candle back toward horizontal or 0 pitch.",
                3.0,
                5.0,
            ),
            Stage(
                "HORIZONTAL_WORLD_YAW",
                "Keep the candle horizontal and avoid twisting it. Physically turn your body to change yaw.",
                3.0,
                8.0,
            ),
            Stage(
                "FINAL_HOLD",
                "Stop moving and hold the candle still for a final baseline.",
                2.0,
                3.0,
            ),
        ),
        key_diagnostics=(
            (
                "HORIZONTAL_AXIS_ROLL",
                "expect synth roll to move strongly while synth yaw stays relatively small",
            ),
            (
                "PITCH_90_AXIS_ROLL",
                "expect synth roll to move again while synth yaw stays relatively small",
            ),
            (
                "HORIZONTAL_WORLD_YAW",
                "expect synth yaw to move strongly while synth roll stays relatively small",
            ),
        ),
    ),
    "pitch-roll-coupling": TraceProtocol(
        name="pitch-roll-coupling",
        description="Focused test for whether pitch motion perturbs roll and whether held roll stays stable through pitch changes.",
        output_prefix="pitch_roll_coupling_trace",
        stages=(
            Stage(
                "HORIZONTAL_BASELINE",
                "Hold the candle horizontal with as little roll as possible and keep it still.",
                4.0,
                4.0,
            ),
            Stage(
                "PITCH_UP_TO_90",
                "From horizontal, raise pitch smoothly to about +90 degrees without intentionally twisting roll.",
                3.0,
                6.0,
            ),
            Stage(
                "PITCH_90_HOLD",
                "Hold near +90 pitch and keep both yaw and roll as steady as possible.",
                2.0,
                3.0,
            ),
            Stage(
                "RETURN_TO_HORIZONTAL",
                "Lower back to horizontal or 0 pitch without intentionally changing roll.",
                3.0,
                6.0,
            ),
            Stage(
                "ROLL_TO_90_HORIZONTAL",
                "While staying horizontal, twist the candle to about +90 roll and then stop.",
                3.0,
                5.0,
            ),
            Stage(
                "ROLL_90_HOLD_HORIZONTAL",
                "Hold that horizontal +90 roll position as steadily as you can.",
                2.0,
                3.0,
            ),
            Stage(
                "PITCH_UP_WITH_ROLL_90",
                "Keep that rolled orientation and raise pitch smoothly to about +90 without adding extra twist.",
                3.0,
                6.0,
            ),
            Stage(
                "PITCH_90_HOLD_WITH_ROLL_90",
                "Hold near +90 pitch while keeping the same rolled orientation.",
                2.0,
                3.0,
            ),
            Stage(
                "RETURN_TO_HORIZONTAL_WITH_ROLL_90",
                "Lower back to horizontal while keeping roughly the same +90 roll.",
                3.0,
                6.0,
            ),
            Stage(
                "FINAL_HOLD",
                "Stop moving and hold the candle still for a final baseline.",
                2.0,
                3.0,
            ),
        ),
        key_diagnostics=(
            (
                "PITCH_UP_TO_90",
                "expect synth pitch to move strongly while synth roll stays relatively small",
            ),
            (
                "RETURN_TO_HORIZONTAL",
                "expect synth pitch to move strongly while synth roll stays relatively small",
            ),
            (
                "ROLL_TO_90_HORIZONTAL",
                "expect synth roll to move strongly while synth pitch stays relatively small",
            ),
            (
                "PITCH_UP_WITH_ROLL_90",
                "expect synth pitch to move strongly while synth roll stays near its held offset",
            ),
            (
                "RETURN_TO_HORIZONTAL_WITH_ROLL_90",
                "expect synth pitch to move strongly while synth roll stays near its held offset",
            ),
        ),
    ),
}


def default_config_path() -> Path:
    return Path(__file__).resolve().parent / "config.json"


def default_output_path(protocol: TraceProtocol) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.cwd() / f"{protocol.output_prefix}_{timestamp}.csv"


def describe_port_holder(port: str) -> str | None:
    try:
        result = subprocess.run(
            ["lsof", "-n", port],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return None

    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if len(lines) <= 1:
        return None

    holders: list[str] = []
    for line in lines[1:4]:
        parts = line.split()
        if len(parts) >= 2:
            holders.append(f"{parts[0]} (pid {parts[1]})")

    if not holders:
        return None

    return ", ".join(holders)


def print_port_busy_message(port: str, holder_summary: str | None = None) -> None:
    print(f"Serial port {port} is busy.")
    if holder_summary:
        print(f"Current holder: {holder_summary}")
    print("Stop the live rig first with ./spacewizard_live.sh stop, then rerun this capture script.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture a guided orientation-diagnostic trace for SpaceWizard yaw, pitch, and roll behavior.",
    )
    parser.add_argument(
        "--protocol",
        choices=sorted(TRACE_PROTOCOLS.keys()),
        default="axis-mapping",
        help="Guided motion protocol to run (default: axis-mapping).",
    )
    parser.add_argument(
        "--port",
        help="Serial port path. If omitted, the script will try to auto-detect the receiver.",
    )
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate (default: 115200)")
    parser.add_argument(
        "--config",
        default=str(default_config_path()),
        help="Config path used for derived metric calculation.",
    )
    parser.add_argument(
        "--output",
        help="CSV output path. Default: <protocol>_trace_<timestamp>.csv in the current directory.",
    )
    parser.add_argument(
        "--status-rate",
        type=float,
        default=6.0,
        help="How often to print live value snapshots in Hz (default: 6)",
    )
    return parser


def resolve_port(port_arg: str | None) -> str | None:
    if port_arg:
        return port_arg
    return find_arduino_port()


def print_stage_header(stage: Stage) -> None:
    print()
    print(f"[{stage.name}] {stage.instruction}")
    print(f"Prep: {stage.prep_s:.1f}s | Capture: {stage.duration_s:.1f}s")


def print_prep_countdown(stage_name: str, seconds_left: int) -> None:
    print(f"[{stage_name}] Start in {seconds_left:>2}s", end="\r", flush=True)


def print_capture_countdown(stage_name: str, seconds_left: int) -> None:
    print(f"[{stage_name}] {seconds_left:>2}s remaining", end="\r", flush=True)


def clear_status_line(width: int = 140) -> None:
    print(" " * width, end="\r", flush=True)


def stage_seconds_left(duration_s: float, elapsed_s: float) -> int:
    remaining = max(0.0, duration_s - elapsed_s)
    if remaining <= 0.0:
        return 0
    return int(remaining) if remaining.is_integer() else int(remaining) + 1


def print_live_status(
    stage_name: str,
    elapsed_ms: int,
    raw_values: dict[str, float | bool],
    metrics: dict[str, object],
) -> None:
    synth_yaw_deg = float(metrics.get("synth_yaw_norm", 0.0)) * 360.0
    print(
        " " * 160,
        end="\r",
        flush=True,
    )
    print(
        f"[{stage_name}] t={elapsed_ms / 1000.0:5.2f}s "
        f"raw(y/p/r)={float(raw_values.get('YAW', 0.0)):>6.1f}/"
        f"{float(raw_values.get('PITCH', 0.0)):>6.1f}/"
        f"{float(raw_values.get('ROLL', 0.0)):>6.1f} "
        f"synth(y/p/r)={synth_yaw_deg:>6.1f}/"
        f"{float(metrics.get('synth_pitch_deg', 0.0)):>6.1f}/"
        f"{float(metrics.get('synth_roll_deg', 0.0)):>6.1f} "
        f"yaw_active={int(bool(metrics.get('synth_yaw_active', False)))} "
        f"horiz={float(metrics.get('synth_horizontal_strength', 0.0)):>4.2f} "
        f"axial={float(metrics.get('synth_axial_gyro_dps', 0.0)):>6.1f}dps"
    )


def unwrap_angle_series(values: list[float], period: float = 360.0) -> list[float]:
    if not values:
        return []

    unwrapped = [values[0]]
    previous = values[0]
    for value in values[1:]:
        delta = ((value - previous + (period / 2.0)) % period) - (period / 2.0)
        unwrapped.append(unwrapped[-1] + delta)
        previous = value
    return unwrapped


def angle_span(values: list[float]) -> float:
    if not values:
        return 0.0
    unwrapped = unwrap_angle_series(values)
    return max(unwrapped) - min(unwrapped)


def value_span(rows: list[dict[str, object]], key: str) -> float:
    if not rows:
        return 0.0
    values = [float(row[key]) for row in rows]
    return max(values) - min(values)


def mean_value(rows: list[dict[str, object]], key: str) -> float:
    if not rows:
        return 0.0
    return sum(float(row[key]) for row in rows) / len(rows)


def mean_abs_value(rows: list[dict[str, object]], key: str) -> float:
    if not rows:
        return 0.0
    return sum(abs(float(row[key])) for row in rows) / len(rows)


def print_stage_summary(stage_name: str, rows: list[dict[str, object]]) -> None:
    raw_yaw_span = angle_span([float(row["raw_yaw_deg"]) for row in rows])
    raw_pitch_span = value_span(rows, "raw_pitch_deg")
    raw_roll_span = value_span(rows, "raw_roll_deg")
    synth_yaw_span = angle_span([float(row["synth_yaw_deg"]) for row in rows])
    synth_pitch_span = value_span(rows, "synth_pitch_deg")
    synth_roll_span = value_span(rows, "synth_roll_deg")
    yaw_active_pct = mean_value(rows, "synth_yaw_active") * 100.0
    horiz_avg = mean_value(rows, "synth_horizontal_strength")
    axial_avg = mean_abs_value(rows, "synth_axial_gyro_dps")

    print(
        f"  {stage_name:>22}: "
        f"raw spans yaw/pitch/roll={raw_yaw_span:>6.1f}/{raw_pitch_span:>6.1f}/{raw_roll_span:>6.1f} | "
        f"synth spans yaw/pitch/roll={synth_yaw_span:>6.1f}/{synth_pitch_span:>6.1f}/{synth_roll_span:>6.1f} | "
        f"yaw active={yaw_active_pct:>5.1f}% | horiz avg={horiz_avg:>4.2f} | axial avg={axial_avg:>6.1f} dps | "
        f"samples={len(rows)}"
    )


def summarize_capture(rows: list[dict[str, object]], protocol: TraceProtocol) -> None:
    print()
    print(f"Capture summary ({protocol.name}):")
    if not rows:
        print("  No parsed samples were captured.")
        return

    rows_by_stage: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        rows_by_stage.setdefault(str(row["stage"]), []).append(row)

    for stage in protocol.stages:
        stage_rows = rows_by_stage.get(stage.name, [])
        if stage_rows:
            print_stage_summary(stage.name, stage_rows)

    if not protocol.key_diagnostics:
        return

    print()
    print("Key diagnostics:")
    for stage_name, expectation in protocol.key_diagnostics:
        stage_rows = rows_by_stage.get(stage_name, [])
        if not stage_rows:
            continue
        print(f"  {stage_name}: {expectation}")
        print_stage_summary(stage_name, stage_rows)


def capture(args: argparse.Namespace) -> int:
    protocol = TRACE_PROTOCOLS[args.protocol]
    port = resolve_port(args.port)
    if not port:
        print("Could not auto-detect the receiver serial port. Pass --port /dev/cu.usbserialXXX.")
        return 1

    port_holder = describe_port_holder(port)
    if port_holder is not None:
        print_port_busy_message(port, port_holder)
        return 1

    output_path = Path(args.output) if args.output else default_output_path(protocol)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    load_result = load_config(args.config)
    config = load_result.config
    impact_state = ImpactState()

    try:
        serial_port = serial.Serial(port=port, baudrate=args.baudrate, timeout=0.05)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {port}: {exc}")
        print_port_busy_message(port, describe_port_holder(port))
        return 1
    except Exception as exc:
        print(f"Failed to open serial port {port}: {exc}")
        return 1

    fieldnames = [
        "elapsed_ms",
        "stage",
        "stage_elapsed_ms",
        "ax_g",
        "ay_g",
        "az_g",
        "gx_dps",
        "gy_dps",
        "gz_dps",
        "raw_yaw_deg",
        "raw_pitch_deg",
        "raw_roll_deg",
        "quat_w",
        "quat_x",
        "quat_y",
        "quat_z",
        "synth_yaw_deg",
        "synth_yaw_norm",
        "synth_pitch_deg",
        "synth_pitch_norm",
        "synth_roll_deg",
        "synth_roll_norm",
        "synth_yaw_active",
        "synth_horizontal_strength",
        "synth_axial_gyro_dps",
        "accel_mag_g",
        "gyro_mag_dps",
        "interaction_gate",
        "interaction_activity",
        "interaction_active",
        "pickup_triggered",
        "impact_triggered",
        "raw_line",
    ]

    captured_rows: list[dict[str, object]] = []

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        print(f"Connected to {port}")
        print(f"Protocol: {protocol.name} - {protocol.description}")
        print(f"Writing parsed capture to: {output_path}")
        print("Follow each prompt. The script will tell you when to start and count down every phase.")

        start_monotonic = time.monotonic()
        stage_index = 0
        current_stage = protocol.stages[stage_index]
        phase = "prep"
        phase_start = start_monotonic
        last_countdown_second: int | None = None
        last_status_time = 0.0
        status_interval = 1.0 / max(args.status_rate, 0.1)

        print_stage_header(current_stage)

        try:
            serial_port.readline()
        except Exception:
            pass

        try:
            while stage_index < len(protocol.stages):
                now = time.monotonic()
                elapsed_total_ms = int((now - start_monotonic) * 1000)
                phase_elapsed_s = now - phase_start
                phase_duration_s = current_stage.prep_s if phase == "prep" else current_stage.duration_s

                seconds_left = stage_seconds_left(phase_duration_s, phase_elapsed_s)
                if seconds_left != last_countdown_second:
                    if phase == "prep":
                        print_prep_countdown(current_stage.name, seconds_left)
                    else:
                        print_capture_countdown(current_stage.name, seconds_left)
                    last_countdown_second = seconds_left

                try:
                    line = serial_port.readline().decode("utf-8", errors="ignore").strip()
                except serial.SerialException as exc:
                    clear_status_line()
                    print()
                    print(f"Serial read failed on {port}: {exc}")
                    print_port_busy_message(port, describe_port_holder(port))
                    print("Partial file saved.")
                    summarize_capture(captured_rows, protocol)
                    return 1
                if line:
                    parsed = parse_serial_line(line)
                    if parsed:
                        metrics = calculate_motion_metrics(parsed, config, impact_state)
                        if metrics and phase == "record":
                            row = {
                                "elapsed_ms": elapsed_total_ms,
                                "stage": current_stage.name,
                                "stage_elapsed_ms": int(phase_elapsed_s * 1000),
                                "ax_g": parsed.get("AX", 0.0),
                                "ay_g": parsed.get("AY", 0.0),
                                "az_g": parsed.get("AZ", 0.0),
                                "gx_dps": parsed.get("GX", 0.0),
                                "gy_dps": parsed.get("GY", 0.0),
                                "gz_dps": parsed.get("GZ", 0.0),
                                "raw_yaw_deg": parsed.get("YAW", 0.0),
                                "raw_pitch_deg": parsed.get("PITCH", 0.0),
                                "raw_roll_deg": parsed.get("ROLL", 0.0),
                                "quat_w": parsed.get("QW", 0.0),
                                "quat_x": parsed.get("QX", 0.0),
                                "quat_y": parsed.get("QY", 0.0),
                                "quat_z": parsed.get("QZ", 0.0),
                                "synth_yaw_deg": float(metrics.get("synth_yaw_norm", 0.0)) * 360.0,
                                "synth_yaw_norm": metrics.get("synth_yaw_norm", 0.0),
                                "synth_pitch_deg": metrics.get("synth_pitch_deg", 0.0),
                                "synth_pitch_norm": metrics.get("synth_pitch_norm", 0.0),
                                "synth_roll_deg": metrics.get("synth_roll_deg", 0.0),
                                "synth_roll_norm": metrics.get("synth_roll_norm", 0.0),
                                "synth_yaw_active": int(bool(metrics.get("synth_yaw_active", False))),
                                "synth_horizontal_strength": metrics.get("synth_horizontal_strength", 0.0),
                                "synth_axial_gyro_dps": metrics.get("synth_axial_gyro_dps", 0.0),
                                "accel_mag_g": metrics.get("accel_magnitude", 0.0),
                                "gyro_mag_dps": metrics.get("gyro_magnitude", 0.0),
                                "interaction_gate": metrics.get("interaction_gate", 0.0),
                                "interaction_activity": metrics.get("interaction_activity", 0.0),
                                "interaction_active": int(bool(metrics.get("interaction_active", False))),
                                "pickup_triggered": int(bool(metrics.get("pickup_triggered", False))),
                                "impact_triggered": int(bool(metrics.get("impact_triggered", False))),
                                "raw_line": line,
                            }
                            writer.writerow(row)
                            captured_rows.append(row)

                            if now - last_status_time >= status_interval:
                                print_live_status(current_stage.name, elapsed_total_ms, parsed, metrics)
                                last_status_time = now
                        elif metrics:
                            # Keep the filtered derived state warm during the prep countdown.
                            pass

                if phase_elapsed_s >= phase_duration_s:
                    clear_status_line()
                    if phase == "prep":
                        phase = "record"
                        phase_start = now
                        last_countdown_second = None
                        print(f"[{current_stage.name}] Start now.")
                    else:
                        stage_index += 1
                        if stage_index >= len(protocol.stages):
                            break
                        current_stage = protocol.stages[stage_index]
                        phase = "prep"
                        phase_start = now
                        last_countdown_second = None
                        print_stage_header(current_stage)

        except KeyboardInterrupt:
            print()
            print("Capture interrupted. Partial file saved.")
            summarize_capture(captured_rows, protocol)
            return 130
        finally:
            serial_port.close()

    print()
    print("Capture complete.")
    print(f"Saved: {output_path}")
    summarize_capture(captured_rows, protocol)
    return 0


def main() -> int:
    return capture(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())