#!/usr/bin/env python3
"""Guided orientation sweep capture for diagnosing pitch/roll control behavior."""

from __future__ import annotations

import sys
from pathlib import Path as _Path

_REPO_ROOT = _Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT))

import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

import serial

from bridge.config import load_config
from bridge.metrics import ImpactState, calculate_motion_metrics
from bridge.serial_input import parse_serial_line


DEFAULT_STAGES = [
    ("REST_FLAT", "Keep the candle motionless at 0 degrees", 2),
    ("PITCH_UP", "Pick up the candle and tilt the top upward toward +90 degrees", 2),
    ("PITCH_DOWN", "Move continuously from +90 degrees down through 0 to -90 degrees", 4),
    ("RETURN_ZERO", "Bring the candle back to 0 degrees", 2),
    ("ROLL_90", "Roll the candle by about 90 degrees and hold that orientation", 2),
    ("ROLLED_PITCH_UP", "In the rolled orientation, tilt the top upward toward +90 degrees", 2),
    ("ROLLED_PITCH_DOWN", "In the rolled orientation, move from +90 through 0 to -90 degrees", 4),
    ("ROLLED_RETURN_ZERO", "Return to 0 degrees and hold still", 2),
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture a staged pitch/roll sweep to CSV for SpaceWizard diagnosis.",
    )
    parser.add_argument("--port", required=True, help="Serial port path (example: /dev/cu.usbserial-10)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate (default: 115200)")
    parser.add_argument(
        "--config",
        default=str(_REPO_ROOT / "config" / "bridge.json"),
        help="Config path used for derived metric calculation (default: config/bridge.json)",
    )
    parser.add_argument(
        "--output",
        help="CSV output path. Default: orientation_sweep_<timestamp>.csv in current directory.",
    )
    return parser


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.cwd() / f"orientation_sweep_{timestamp}.csv"


def print_stage_header(stage_name: str, instruction: str, duration_s: int) -> None:
    print()
    print(f"[{stage_name}] {instruction}")
    print(f"Duration: {duration_s}s")


def print_stage_countdown(stage_name: str, seconds_left: int) -> None:
    print(f"[{stage_name}] {seconds_left:>2}s remaining", end="\r", flush=True)


def summarize_capture(rows: list[dict[str, object]]) -> None:
    print()
    print("Capture summary:")
    stage_names = []
    for row in rows:
        stage = str(row["stage"])
        if stage not in stage_names:
            stage_names.append(stage)

    for stage in stage_names:
        stage_rows = [row for row in rows if row["stage"] == stage]
        if not stage_rows:
            continue
        pitch_values = [float(row["pitch_deg"]) for row in stage_rows]
        roll_values = [float(row["roll_deg"]) for row in stage_rows]
        print(
            f"  {stage:>18}: "
            f"pitch {min(pitch_values):>7.1f}..{max(pitch_values):>7.1f} deg | "
            f"roll {min(roll_values):>7.1f}..{max(roll_values):>7.1f} deg | "
            f"samples={len(stage_rows)}"
        )


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
        "ax_g",
        "ay_g",
        "az_g",
        "gx_dps",
        "gy_dps",
        "gz_dps",
        "yaw_deg",
        "pitch_deg",
        "roll_deg",
        "accel_mag_g",
        "gyro_mag_dps",
        "interaction_gate",
        "interaction_activity",
        "pickup_triggered",
        "impact_triggered",
        "raw_line",
    ]

    captured_rows: list[dict[str, object]] = []

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        print(f"Connected to {args.port}")
        print(f"Writing parsed capture to: {output_path}")
        print("Follow the stage prompts. Raw lines are being recorded to CSV.")

        start_monotonic = time.monotonic()
        stage_idx = 0
        stage_start = start_monotonic
        stage_name, instruction, stage_duration = DEFAULT_STAGES[stage_idx]
        print_stage_header(stage_name, instruction, stage_duration)
        last_countdown_second = None

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
                            row = {
                                "elapsed_ms": elapsed_total_ms,
                                "stage": stage_name,
                                "ax_g": parsed.get("AX", 0.0),
                                "ay_g": parsed.get("AY", 0.0),
                                "az_g": parsed.get("AZ", 0.0),
                                "gx_dps": parsed.get("GX", 0.0),
                                "gy_dps": parsed.get("GY", 0.0),
                                "gz_dps": parsed.get("GZ", 0.0),
                                "yaw_deg": parsed.get("YAW", 0.0),
                                "pitch_deg": parsed.get("PITCH", 0.0),
                                "roll_deg": parsed.get("ROLL", 0.0),
                                "accel_mag_g": metrics.get("accel_magnitude", 0.0),
                                "gyro_mag_dps": metrics.get("gyro_magnitude", 0.0),
                                "interaction_gate": metrics.get("interaction_gate", 0.0),
                                "interaction_activity": metrics.get("interaction_activity", 0.0),
                                "pickup_triggered": int(bool(metrics.get("pickup_triggered", False))),
                                "impact_triggered": int(bool(metrics.get("impact_triggered", False))),
                                "raw_line": line,
                            }
                            writer.writerow(row)
                            captured_rows.append(row)

                if elapsed_stage >= stage_duration:
                    print(" " * 70, end="\r", flush=True)
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
            summarize_capture(captured_rows)
            return 130
        finally:
            serial_port.close()

    print()
    print("Capture complete.")
    print(f"Saved: {output_path}")
    summarize_capture(captured_rows)
    return 0


def main() -> int:
    return capture(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())