#!/usr/bin/env python3
"""Guided raw serial capture for pickup/move/rest interaction tuning."""

from __future__ import annotations

import sys
from pathlib import Path as _Path

_REPO_ROOT = _Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT))

import argparse
from datetime import datetime
from pathlib import Path
import time

import serial


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture raw MPU6050 serial data to file with guided timing prompts.",
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port path (example: /dev/cu.usbserial-2130)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Serial baudrate (default: 115200)",
    )
    parser.add_argument(
        "--rest-start",
        type=int,
        default=5,
        help="Seconds to keep object still at start (default: 5)",
    )
    parser.add_argument(
        "--move",
        type=int,
        default=10,
        help="Seconds to pick up and move object (default: 10)",
    )
    parser.add_argument(
        "--rest-end",
        type=int,
        default=5,
        help="Seconds to keep object still at end (default: 5)",
    )
    parser.add_argument(
        "--output",
        help=(
            "Output file path. Default: pickup_session_<timestamp>.log "
            "in current directory."
        ),
    )
    return parser


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.cwd() / f"pickup_session_{timestamp}.log"


def print_stage_header(stage_name: str, instruction: str, duration_s: int) -> None:
    print()
    print(f"[{stage_name}] {instruction}")
    print(f"Duration: {duration_s}s")


def print_stage_countdown(stage_name: str, seconds_left: int) -> None:
    print(
        f"[{stage_name}] {seconds_left:>2}s remaining",
        end="\r",
        flush=True,
    )


def capture_session(args: argparse.Namespace) -> int:
    output_path = Path(args.output) if args.output else default_output_path()

    stages = [
        ("REST_START", "Keep object still on the ground", max(0, args.rest_start)),
        ("MOVE", "Pick up the object and move it around", max(0, args.move)),
        ("REST_END", "Put object down and keep it still", max(0, args.rest_end)),
    ]

    try:
        serial_port = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            timeout=0.05,
        )
    except Exception as exc:
        print(f"Failed to open serial port {args.port}: {exc}")
        return 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as out:
        out.write("# SpaceWizard guided pickup session capture\n")
        out.write(f"# port={args.port} baudrate={args.baudrate}\n")
        out.write(
            f"# rest_start_s={stages[0][2]} move_s={stages[1][2]} rest_end_s={stages[2][2]}\n"
        )
        out.write("# elapsed_ms\tstage\traw_line\n")

        print(f"Connected to {args.port}")
        print(f"Writing raw data to: {output_path}")
        print("Raw sensor lines will not be printed to console.")

        start_monotonic = time.monotonic()
        stage_start = start_monotonic
        stage_idx = 0
        stage_name, instruction, stage_duration = stages[stage_idx]
        print_stage_header(stage_name, instruction, stage_duration)
        last_countdown_second = None

        # Skip startup line once, same behavior as bridge app.
        try:
            serial_port.readline()
        except Exception:
            pass

        try:
            while stage_idx < len(stages):
                now = time.monotonic()
                elapsed_stage = now - stage_start
                elapsed_total_ms = int((now - start_monotonic) * 1000)

                seconds_left = int(max(0, stage_duration - elapsed_stage))
                if seconds_left != last_countdown_second:
                    print_stage_countdown(stage_name, seconds_left)
                    last_countdown_second = seconds_left

                # Capture any available serial line to file only.
                line = serial_port.readline().decode("utf-8", errors="ignore").strip()
                if line and "AX=" in line and "GX=" in line:
                    out.write(f"{elapsed_total_ms}\t{stage_name}\t{line}\n")

                if elapsed_stage >= stage_duration:
                    print(" " * 60, end="\r", flush=True)
                    stage_idx += 1
                    if stage_idx >= len(stages):
                        break
                    stage_start = now
                    stage_name, instruction, stage_duration = stages[stage_idx]
                    print_stage_header(stage_name, instruction, stage_duration)
                    last_countdown_second = None

            print()
            print("Capture complete.")
            print(f"Saved: {output_path}")
            return 0
        except KeyboardInterrupt:
            print()
            print("Capture interrupted by user.")
            print(f"Partial file saved: {output_path}")
            return 130
        finally:
            serial_port.close()


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return capture_session(args)


if __name__ == "__main__":
    raise SystemExit(main())

