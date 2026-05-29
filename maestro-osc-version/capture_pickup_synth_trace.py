#!/usr/bin/env python3
"""Guided pickup trace focused on derived synth control values."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import time

import serial

from bridge.config import load_config
from bridge.metrics import ImpactState, calculate_motion_metrics
from bridge.serial_input import parse_serial_line


DEFAULT_STAGES = [
    ("REST_FLAT", "Leave the candle flat and motionless on the table.", 2.0),
    ("LIFT_TO_PITCH_90", "Pick it up and tilt the top upward toward +90 degrees.", 3.5),
    ("HOLD_PITCH_90", "Hold near +90 degrees.", 2.5),
    ("RETURN_TO_ZERO", "Bring it back toward 0 degrees while still holding it.", 3.0),
    ("PLACE_FLAT", "Place it back flat on the table and let it settle long enough to observe gate-off.", 8.0),
]


@dataclass
class GateTransition:
    elapsed_ms: int
    stage: str
    gate_value: float


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture a guided pickup-to-tilt trace and log derived synth values.",
    )
    parser.add_argument("--port", required=True, help="Serial port path, for example /dev/cu.usbserial-10")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate (default: 115200)")
    parser.add_argument(
        "--config",
        default="config.json",
        help="Config path used for derived metric calculation (default: config.json)",
    )
    parser.add_argument(
        "--output",
        help="CSV output path. Default: pickup_synth_trace_<timestamp>.csv in the current directory.",
    )
    parser.add_argument(
        "--status-rate",
        type=float,
        default=6.0,
        help="How often to print live synth-value snapshots in Hz (default: 6)",
    )
    return parser


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.cwd() / f"pickup_synth_trace_{timestamp}.csv"


def print_stage_header(stage_name: str, instruction: str, duration_s: float) -> None:
    print()
    print(f"[{stage_name}] {instruction}")
    print(f"Duration: {duration_s:.1f}s")


def print_stage_countdown(stage_name: str, seconds_left: int) -> None:
    print(f"[{stage_name}] {seconds_left:>2}s remaining", end="\r", flush=True)


def print_live_status(stage_name: str, elapsed_ms: int, metrics: dict[str, object]) -> None:
    print(
        " " * 100,
        end="\r",
        flush=True,
    )
    print(
        f"[{stage_name}] t={elapsed_ms / 1000.0:5.2f}s "
        f"synth_pitch={float(metrics.get('synth_pitch_deg', 0.0)):>6.1f}deg "
        f"synth_roll={float(metrics.get('synth_roll_deg', 0.0)):>6.1f}deg "
        f"pitch_norm={float(metrics.get('synth_pitch_norm', 0.0)):>5.2f} "
        f"roll_norm={float(metrics.get('synth_roll_norm', 0.0)):>5.2f} "
        f"gate={float(metrics.get('interaction_gate', 0.0)):>4.2f} "
        f"activity={float(metrics.get('interaction_activity', 0.0)):>4.2f} "
        f"pickup={int(bool(metrics.get('pickup_triggered', False)))}"
    )


def summarize_capture(
    rows: list[dict[str, object]],
    gate_transitions: list[GateTransition],
) -> None:
    print()
    print("Capture summary:")
    if not rows:
        print("  No parsed samples were captured.")
        return

    stage_names: list[str] = []
    for row in rows:
        stage = str(row["stage"])
        if stage not in stage_names:
            stage_names.append(stage)

    for stage in stage_names:
        stage_rows = [row for row in rows if row["stage"] == stage]
        if not stage_rows:
            continue
        synth_pitch_values = [float(row["synth_pitch_deg"]) for row in stage_rows]
        synth_roll_values = [float(row["synth_roll_deg"]) for row in stage_rows]
        gate_values = [float(row["interaction_gate"]) for row in stage_rows]
        pickup_frames = sum(int(row["pickup_triggered"]) for row in stage_rows)
        print(
            f"  {stage:>16}: "
            f"synth pitch {min(synth_pitch_values):>6.1f}..{max(synth_pitch_values):>6.1f} deg | "
            f"synth roll {min(synth_roll_values):>6.1f}..{max(synth_roll_values):>6.1f} deg | "
            f"gate avg {sum(gate_values) / len(gate_values):>4.2f} | "
            f"pickup frames={pickup_frames:>2} | samples={len(stage_rows)}"
        )

    pickup_events = [row for row in rows if int(row["pickup_triggered"]) == 1]
    if pickup_events:
        first_pickup = pickup_events[0]
        print(
            "  First pickup trigger: "
            f"t={int(first_pickup['elapsed_ms']) / 1000.0:.2f}s in {first_pickup['stage']}"
        )
    else:
        print("  First pickup trigger: none")

    if gate_transitions:
        print("  Gate transitions:")
        for transition in gate_transitions:
            print(
                f"    t={transition.elapsed_ms / 1000.0:5.2f}s "
                f"stage={transition.stage:<16} gate={transition.gate_value:.2f}"
            )
    else:
        print("  Gate transitions: none")


def capture(args: argparse.Namespace) -> int:
    output_path = Path(args.output) if args.output else default_output_path()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    load_result = load_config(args.config)
    config = load_result.config
    impact_state = ImpactState()

    try:
        serial_port = serial.Serial(port=args.port, baudrate=args.baudrate, timeout=0.05)
    except Exception as exc:
        print(f"Failed to open serial port {args.port}: {exc}")
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
        "yaw_deg",
        "pitch_deg",
        "roll_deg",
        "synth_pitch_deg",
        "synth_roll_deg",
        "synth_pitch_norm",
        "synth_roll_norm",
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
    gate_transitions: list[GateTransition] = []
    previous_gate: float | None = None

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        print(f"Connected to {args.port}")
        print(f"Writing parsed capture to: {output_path}")
        print("Follow the stage prompts. Derived synth values are logged to CSV.")

        start_monotonic = time.monotonic()
        stage_idx = 0
        stage_start = start_monotonic
        stage_name, instruction, stage_duration = DEFAULT_STAGES[stage_idx]
        print_stage_header(stage_name, instruction, stage_duration)
        last_countdown_second: int | None = None
        last_status_time = 0.0
        status_interval = 1.0 / max(args.status_rate, 0.1)

        try:
            serial_port.readline()
        except Exception:
            pass

        try:
            while stage_idx < len(DEFAULT_STAGES):
                now = time.monotonic()
                elapsed_stage = now - stage_start
                elapsed_total_ms = int((now - start_monotonic) * 1000)

                seconds_left = int(max(0, stage_duration - elapsed_stage))
                if seconds_left != last_countdown_second:
                    print_stage_countdown(stage_name, seconds_left)
                    last_countdown_second = seconds_left

                line = serial_port.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    parsed = parse_serial_line(line)
                    if parsed:
                        metrics = calculate_motion_metrics(parsed, config, impact_state)
                        if metrics:
                            gate_value = float(metrics.get("interaction_gate", 0.0))
                            if previous_gate is None or abs(gate_value - previous_gate) >= 0.5:
                                gate_transitions.append(
                                    GateTransition(
                                        elapsed_ms=elapsed_total_ms,
                                        stage=stage_name,
                                        gate_value=gate_value,
                                    )
                                )
                                previous_gate = gate_value

                            row = {
                                "elapsed_ms": elapsed_total_ms,
                                "stage": stage_name,
                                "stage_elapsed_ms": int(elapsed_stage * 1000),
                                "ax_g": parsed.get("AX", 0.0),
                                "ay_g": parsed.get("AY", 0.0),
                                "az_g": parsed.get("AZ", 0.0),
                                "gx_dps": parsed.get("GX", 0.0),
                                "gy_dps": parsed.get("GY", 0.0),
                                "gz_dps": parsed.get("GZ", 0.0),
                                "yaw_deg": parsed.get("YAW", 0.0),
                                "pitch_deg": parsed.get("PITCH", 0.0),
                                "roll_deg": parsed.get("ROLL", 0.0),
                                "synth_pitch_deg": metrics.get("synth_pitch_deg", 0.0),
                                "synth_roll_deg": metrics.get("synth_roll_deg", 0.0),
                                "synth_pitch_norm": metrics.get("synth_pitch_norm", 0.0),
                                "synth_roll_norm": metrics.get("synth_roll_norm", 0.0),
                                "accel_mag_g": metrics.get("accel_magnitude", 0.0),
                                "gyro_mag_dps": metrics.get("gyro_magnitude", 0.0),
                                "interaction_gate": gate_value,
                                "interaction_activity": metrics.get("interaction_activity", 0.0),
                                "interaction_active": int(bool(metrics.get("interaction_active", False))),
                                "pickup_triggered": int(bool(metrics.get("pickup_triggered", False))),
                                "impact_triggered": int(bool(metrics.get("impact_triggered", False))),
                                "raw_line": line,
                            }
                            writer.writerow(row)
                            captured_rows.append(row)

                            if now - last_status_time >= status_interval:
                                print_live_status(stage_name, elapsed_total_ms, metrics)
                                last_status_time = now

                if elapsed_stage >= stage_duration:
                    print(" " * 100, end="\r", flush=True)
                    stage_idx += 1
                    if stage_idx >= len(DEFAULT_STAGES):
                        break
                    stage_start = now
                    stage_name, instruction, stage_duration = DEFAULT_STAGES[stage_idx]
                    print_stage_header(stage_name, instruction, stage_duration)
                    last_countdown_second = None

        except KeyboardInterrupt:
            print()
            print("Capture interrupted. Partial file saved.")
            summarize_capture(captured_rows, gate_transitions)
            return 130
        finally:
            serial_port.close()

    print()
    print("Capture complete.")
    print(f"Saved: {output_path}")
    summarize_capture(captured_rows, gate_transitions)
    return 0


def main() -> int:
    return capture(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())