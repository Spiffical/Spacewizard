#!/usr/bin/env python3
"""Guided timing probe for SpaceWizard serial, bridge, and OSC stages."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import shutil
import socket
import statistics
import subprocess
import threading
import time

import serial
from pythonosc.udp_client import SimpleUDPClient

from bridge.config import load_config
from bridge.metrics import ImpactState, calculate_motion_metrics
from bridge.osc import send_synth_control_messages, send_synth_status_messages
from bridge.serial_input import find_arduino_port, parse_serial_line


@dataclass(frozen=True)
class Stage:
    name: str
    instruction: str
    cue: str
    prep_s: float
    duration_s: float
    movement_stage: bool = False
    baseline_stage: str | None = None


@dataclass(frozen=True)
class StageLatencySummary:
    stage_name: str
    raw_motion_ms: int | None
    synth_pitch_ms: int | None
    interaction_active_ms: int | None
    pickup_trigger_ms: int | None


TIMING_PROTOCOL = (
    Stage(
        name="HORIZONTAL_REST",
        instruction="Hold the candle horizontal and keep it still.",
        cue="Hold the candle horizontal and keep it still.",
        prep_s=4.0,
        duration_s=4.0,
    ),
    Stage(
        name="FAST_UPRIGHT",
        instruction="Move quickly from horizontal to upright, then freeze there.",
        cue="Move to upright now and freeze.",
        prep_s=3.0,
        duration_s=4.0,
        movement_stage=True,
        baseline_stage="HORIZONTAL_REST",
    ),
    Stage(
        name="UPRIGHT_REST",
        instruction="Hold the candle upright and still.",
        cue="Hold upright and keep still.",
        prep_s=2.0,
        duration_s=3.0,
    ),
    Stage(
        name="FAST_HORIZONTAL",
        instruction="Move quickly back to horizontal, then freeze there.",
        cue="Move back to horizontal now and freeze.",
        prep_s=3.0,
        duration_s=4.0,
        movement_stage=True,
        baseline_stage="UPRIGHT_REST",
    ),
    Stage(
        name="RAPID_REPEATS",
        instruction="Quickly alternate between horizontal and upright several times.",
        cue="Repeat quickly between horizontal and upright now.",
        prep_s=3.0,
        duration_s=8.0,
        movement_stage=True,
    ),
    Stage(
        name="FINAL_REST",
        instruction="Stop moving and hold the candle still for the final baseline.",
        cue="Final rest. Hold the candle still.",
        prep_s=2.0,
        duration_s=4.0,
    ),
)


def default_config_path() -> Path:
    return Path(__file__).resolve().parent / "config.json"


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.cwd() / f"timing_probe_{timestamp}.csv"


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    raw_index = (pct / 100.0) * (len(ordered) - 1)
    low_index = int(raw_index)
    high_index = min(len(ordered) - 1, low_index + 1)
    blend = raw_index - low_index
    return ordered[low_index] * (1.0 - blend) + ordered[high_index] * blend


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
    for line in lines[1:5]:
        parts = line.split()
        if len(parts) >= 2:
            holders.append(f"{parts[0]} (pid {parts[1]})")

    return ", ".join(holders) if holders else None


def print_port_busy_message(port: str, holder_summary: str | None = None) -> None:
    print(f"Serial port {port} is busy.")
    if holder_summary:
        print(f"Current holder: {holder_summary}")
    print("Stop the live rig first with ./spacewizard_live.sh stop, then rerun this timing probe.")


class UdpSink:
    """Discard UDP packets so local OSC send timing can be measured without SC running."""

    def __init__(self) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("127.0.0.1", 0))
        self.host = "127.0.0.1"
        self.port = self._socket.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        self._socket.settimeout(0.2)
        while not self._stop.is_set():
            try:
                self._socket.recvfrom(8192)
            except TimeoutError:
                continue
            except OSError:
                break

    def close(self) -> None:
        self._stop.set()
        try:
            self._socket.close()
        except OSError:
            pass
        self._thread.join(timeout=0.5)


def speak(text: str, enabled: bool) -> None:
    if not enabled or not text:
        return
    if shutil.which("say") is None:
        return
    try:
        subprocess.Popen(
            ["say", text],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        return


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture timing stats for the SpaceWizard receiver, Python bridge stages, and OSC send overhead.",
    )
    parser.add_argument(
        "--port",
        help="Serial port path. If omitted, the script will auto-detect the receiver.",
    )
    parser.add_argument("--baudrate", type=int, default=230400, help="Serial baudrate (default: 230400)")
    parser.add_argument(
        "--config",
        default=str(default_config_path()),
        help="Bridge config path used for derived metric calculation.",
    )
    parser.add_argument(
        "--output",
        help="CSV output path. Default: timing_probe_<timestamp>.csv in the current directory.",
    )
    parser.add_argument(
        "--status-rate",
        type=float,
        default=4.0,
        help="How often to print live timing snapshots in Hz (default: 4).",
    )
    parser.add_argument(
        "--no-speech",
        action="store_true",
        help="Disable spoken stage cues on macOS.",
    )
    return parser


def estimate_serial_wire_ms(line_bytes: int, baudrate: int) -> float:
    return (line_bytes * 10.0 * 1000.0) / max(baudrate, 1)


def median_tail(values: list[float], sample_count: int = 20) -> float | None:
    if not values:
        return None
    return statistics.median(values[-sample_count:])


def stage_rows(rows: list[dict[str, object]], stage_name: str) -> list[dict[str, object]]:
    return [row for row in rows if row["stage"] == stage_name]


def summarize_stage_latency(
    rows: list[dict[str, object]],
    config: dict[str, object],
) -> list[StageLatencySummary]:
    accel_on_g = float(config.get("interaction_mode", {}).get("accel_on_g", 0.035))
    gyro_on_dps = float(config.get("interaction_mode", {}).get("gyro_on_dps", 6.0))
    pitch_threshold_deg = 12.0

    summaries: list[StageLatencySummary] = []
    for stage in TIMING_PROTOCOL:
        if not stage.movement_stage:
            continue

        current_stage_rows = stage_rows(rows, stage.name)
        if not current_stage_rows:
            summaries.append(StageLatencySummary(stage.name, None, None, None, None))
            continue

        baseline_pitch = None
        if stage.baseline_stage:
            baseline_stage_rows = stage_rows(rows, stage.baseline_stage)
            baseline_pitch = median_tail(
                [float(row["synth_pitch_deg"]) for row in baseline_stage_rows]
            )

        stage_starts_active = int(current_stage_rows[0]["interaction_active"]) == 1
        stage_starts_with_pickup = int(current_stage_rows[0]["pickup_triggered"]) == 1

        raw_motion_ms = None
        synth_pitch_ms = None
        interaction_active_ms = None
        pickup_trigger_ms = None

        for row in current_stage_rows:
            frame_time_ms = int(row["stage_elapsed_ms"])
            accel_delta_g = abs(float(row["interaction_deviation_g"]))
            gyro_mag_dps = float(row["gyro_mag_dps"])
            if raw_motion_ms is None and (
                accel_delta_g >= accel_on_g or gyro_mag_dps >= gyro_on_dps
            ):
                raw_motion_ms = frame_time_ms

            if baseline_pitch is not None and synth_pitch_ms is None:
                if abs(float(row["synth_pitch_deg"]) - baseline_pitch) >= pitch_threshold_deg:
                    synth_pitch_ms = frame_time_ms

            if (
                not stage_starts_active
                and interaction_active_ms is None
                and int(row["interaction_active"]) == 1
            ):
                interaction_active_ms = frame_time_ms

            if (
                not stage_starts_with_pickup
                and pickup_trigger_ms is None
                and int(row["pickup_triggered"]) == 1
            ):
                pickup_trigger_ms = frame_time_ms

        summaries.append(
            StageLatencySummary(
                stage_name=stage.name,
                raw_motion_ms=raw_motion_ms,
                synth_pitch_ms=(
                    None
                    if raw_motion_ms is None or synth_pitch_ms is None
                    else max(0, synth_pitch_ms - raw_motion_ms)
                ),
                interaction_active_ms=(
                    None
                    if raw_motion_ms is None or interaction_active_ms is None
                    else max(0, interaction_active_ms - raw_motion_ms)
                ),
                pickup_trigger_ms=(
                    None
                    if raw_motion_ms is None or pickup_trigger_ms is None
                    else max(0, pickup_trigger_ms - raw_motion_ms)
                ),
            )
        )

    return summaries


def build_summary_text(
    rows: list[dict[str, object]],
    config: dict[str, object],
    baudrate: int,
) -> str:
    if not rows:
        return "No parsed samples were captured."

    frame_gaps_ms = [
        float(rows[index]["frame_gap_ms"])
        for index in range(1, len(rows))
        if rows[index]["stage"] == rows[index - 1]["stage"]
        and float(rows[index]["stage_elapsed_ms"]) >= 1000.0
        and float(rows[index]["frame_gap_ms"]) > 0.0
    ]
    parse_ms = [float(row["parse_ms"]) for row in rows]
    metrics_ms = [float(row["metrics_ms"]) for row in rows]
    synth_osc_ms = [float(row["synth_osc_ms"]) for row in rows]
    status_osc_ms = [float(row["status_osc_ms"]) for row in rows]
    line_bytes = [int(row["line_bytes"]) for row in rows]
    serial_wire_ms = [estimate_serial_wire_ms(value, baudrate) for value in line_bytes]
    effective_hz = 1000.0 / max(statistics.fmean(frame_gaps_ms), 1e-6)
    long_gap_20 = sum(1 for value in frame_gaps_ms if value >= 20.0)
    long_gap_40 = sum(1 for value in frame_gaps_ms if value >= 40.0)
    long_gap_80 = sum(1 for value in frame_gaps_ms if value >= 80.0)
    stage_latency = summarize_stage_latency(rows, config)
    interaction_cfg = config.get("interaction_mode", {})
    synth_cfg = config.get("synth_settings", {})

    lines = [
        f"Captured {len(rows)} frames at {effective_hz:.1f} Hz steady-state after the first second of each stage.",
        "",
        "Receiver / serial path:",
        (
            f"  Frame gap ms: mean {statistics.fmean(frame_gaps_ms):.2f}, "
            f"p95 {percentile(frame_gaps_ms, 95):.2f}, max {max(frame_gaps_ms):.2f}"
        ),
        (
            f"  Line bytes: mean {statistics.fmean(line_bytes):.1f}, "
            f"estimated wire time at {baudrate} baud: mean {statistics.fmean(serial_wire_ms):.2f} ms"
        ),
        (
            f"  Long gaps: >=20 ms {long_gap_20}, >=40 ms {long_gap_40}, >=80 ms {long_gap_80}"
        ),
        "",
        "Python bridge stage cost on this machine:",
        (
            f"  parse_serial_line: mean {statistics.fmean(parse_ms):.3f} ms, "
            f"p95 {percentile(parse_ms, 95):.3f} ms"
        ),
        (
            f"  calculate_motion_metrics: mean {statistics.fmean(metrics_ms):.3f} ms, "
            f"p95 {percentile(metrics_ms, 95):.3f} ms"
        ),
        (
            f"  synth OSC send: mean {statistics.fmean(synth_osc_ms):.3f} ms, "
            f"p95 {percentile(synth_osc_ms, 95):.3f} ms"
        ),
        (
            f"  status OSC send: mean {statistics.fmean(status_osc_ms):.3f} ms, "
            f"p95 {percentile(status_osc_ms, 95):.3f} ms"
        ),
        "",
        "Configured latency knobs currently in the system:",
        (
            f"  interaction start hold: {int(interaction_cfg.get('start_hold_ms', 90))} ms"
        ),
        (
            f"  interaction rest hold: {int(interaction_cfg.get('rest_hold_ms', 3000))} ms"
        ),
        (
            f"  synth status OSC rate: {float(synth_cfg.get('status_rate_hz', 20.0)):.1f} Hz"
        ),
        (
            "  SC motion fade-in/out: 3.2 s / 5.0 s"
        ),
        (
            "  smooth-sine minimum note interval: about 100 to 160 ms depending on activity"
        ),
        "",
        "Movement-stage response after raw motion onset:",
    ]

    for item in stage_latency:
        raw_label = "n/a" if item.raw_motion_ms is None else f"{item.raw_motion_ms} ms from stage start"
        synth_label = "n/a" if item.synth_pitch_ms is None else f"{item.synth_pitch_ms} ms"
        gate_label = "n/a" if item.interaction_active_ms is None else f"{item.interaction_active_ms} ms"
        pickup_label = "n/a" if item.pickup_trigger_ms is None else f"{item.pickup_trigger_ms} ms"
        lines.append(
            f"  {item.stage_name}: raw onset {raw_label}, synth pitch response {synth_label}, interaction_active {gate_label}, pickup_trigger {pickup_label}"
        )

    lines.extend(
        [
            "",
            "Interpretation guide:",
            "  If frame gaps are already around 20 to 25 ms or worse, the receiver-side ASCII serial path is a larger bottleneck than Python.",
            "  If parse/metrics/OSC times stay below 1 ms while frame gaps are tens of ms, rewriting the bridge in C++ or Rust will not be the first win.",
            "  If interaction_active consistently trails raw motion by about 90 ms, that is the configured gate hold, not a transport failure.",
            "  If audible bloom feels slow even when serial and Python are fast, the 3.2 s SuperCollider motion fade-in is the dominant intentional delay.",
        ]
    )

    return "\n".join(lines)


def print_live_status(
    stage_name: str,
    recent_rows: list[dict[str, object]],
    latest_row: dict[str, object],
) -> None:
    recent_gaps = [float(row["frame_gap_ms"]) for row in recent_rows if float(row["frame_gap_ms"]) > 0.0]
    recent_parse = [float(row["parse_ms"]) for row in recent_rows]
    recent_metrics = [float(row["metrics_ms"]) for row in recent_rows]
    fps = 0.0
    if len(recent_rows) >= 2:
        window_ms = int(recent_rows[-1]["elapsed_ms"]) - int(recent_rows[0]["elapsed_ms"])
        fps = (len(recent_rows) * 1000.0 / max(window_ms, 1))

    gap_label = f"{statistics.fmean(recent_gaps):5.1f}" if recent_gaps else "  n/a"
    p95_label = f"{percentile(recent_gaps, 95):5.1f}" if recent_gaps else "  n/a"
    parse_label = f"{statistics.fmean(recent_parse):5.3f}" if recent_parse else "  n/a"
    metrics_label = f"{statistics.fmean(recent_metrics):5.3f}" if recent_metrics else "  n/a"

    print(
        " " * 140,
        end="\r",
        flush=True,
    )
    print(
        f"[{stage_name}] "
        f"t={int(latest_row['elapsed_ms']) / 1000.0:5.2f}s "
        f"fps={fps:5.1f} "
        f"gap_mean={gap_label}ms "
        f"gap_p95={p95_label}ms "
        f"parse={parse_label}ms "
        f"metrics={metrics_label}ms "
        f"gyro={float(latest_row['gyro_mag_dps']):6.2f}dps "
        f"pitch={float(latest_row['synth_pitch_deg']):6.1f}deg "
        f"active={int(latest_row['interaction_active'])} "
        f"pickup={int(latest_row['pickup_triggered'])}",
        end="\r",
        flush=True,
    )


def capture(args: argparse.Namespace) -> int:
    port = args.port or find_arduino_port()
    if not port:
        print("Could not auto-detect the receiver serial port.")
        return 1

    holder_summary = describe_port_holder(port)
    if holder_summary:
        print_port_busy_message(port, holder_summary)
        return 1

    output_path = Path(args.output) if args.output else default_output_path()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path = output_path.with_suffix(".summary.txt")

    load_result = load_config(args.config)
    config = load_result.config
    impact_state = ImpactState()
    udp_sink = UdpSink()
    osc_client = SimpleUDPClient(udp_sink.host, udp_sink.port)

    try:
        serial_port = serial.Serial(port=port, baudrate=args.baudrate, timeout=0.05)
    except Exception as exc:
        udp_sink.close()
        print(f"Failed to open serial port {port}: {exc}")
        return 1

    fieldnames = [
        "elapsed_ms",
        "stage",
        "stage_elapsed_ms",
        "line_bytes",
        "frame_gap_ms",
        "parse_ms",
        "metrics_ms",
        "synth_osc_ms",
        "status_osc_ms",
        "interaction_deviation_g",
        "accel_mag_g",
        "gyro_mag_dps",
        "orientation_pitch_deg",
        "orientation_roll_deg",
        "synth_pitch_deg",
        "synth_roll_deg",
        "interaction_active",
        "pickup_triggered",
        "impact_triggered",
        "raw_line",
    ]
    rows: list[dict[str, object]] = []
    last_frame_time = None
    last_status_print = 0.0
    status_interval_s = 1.0 / max(args.status_rate, 0.1)
    speech_enabled = not args.no_speech

    print(f"Connected to {port}")
    print(f"Writing timing capture to {output_path}")
    print("Listen for spoken cues. The script will guide the movement stages and print timing stats live.")

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        try:
            serial_port.reset_input_buffer()
        except Exception:
            pass

        start_time = time.perf_counter()

        try:
            for stage in TIMING_PROTOCOL:
                print()
                print(f"Next stage: {stage.name}")
                print(stage.instruction)
                for second in range(int(stage.prep_s), 0, -1):
                    print(f"Starting in {second:>2}s", end="\r", flush=True)
                    time.sleep(1.0)

                print(" " * 80, end="\r", flush=True)
                try:
                    serial_port.reset_input_buffer()
                except Exception:
                    pass
                print(f"[{stage.name}] {stage.instruction}")
                speak(stage.cue, speech_enabled)
                stage_start = time.perf_counter()

                while (time.perf_counter() - stage_start) < stage.duration_s:
                    line = serial_port.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue

                    frame_time = time.perf_counter()
                    elapsed_ms = int((frame_time - start_time) * 1000.0)
                    stage_elapsed_ms = int((frame_time - stage_start) * 1000.0)
                    frame_gap_ms = 0.0 if last_frame_time is None else (frame_time - last_frame_time) * 1000.0
                    last_frame_time = frame_time
                    line_bytes = len((line + "\r\n").encode("utf-8"))

                    parse_start = time.perf_counter()
                    parsed = parse_serial_line(line)
                    parse_ms = (time.perf_counter() - parse_start) * 1000.0
                    if not parsed:
                        continue

                    metrics_start = time.perf_counter()
                    metrics = calculate_motion_metrics(parsed, config, impact_state)
                    metrics_ms = (time.perf_counter() - metrics_start) * 1000.0
                    if not metrics:
                        continue

                    synth_send_start = time.perf_counter()
                    send_synth_control_messages(metrics, osc_client)
                    synth_osc_ms = (time.perf_counter() - synth_send_start) * 1000.0

                    status_send_start = time.perf_counter()
                    send_synth_status_messages(metrics, osc_client)
                    status_osc_ms = (time.perf_counter() - status_send_start) * 1000.0

                    row = {
                        "elapsed_ms": elapsed_ms,
                        "stage": stage.name,
                        "stage_elapsed_ms": stage_elapsed_ms,
                        "line_bytes": line_bytes,
                        "frame_gap_ms": frame_gap_ms,
                        "parse_ms": parse_ms,
                        "metrics_ms": metrics_ms,
                        "synth_osc_ms": synth_osc_ms,
                        "status_osc_ms": status_osc_ms,
                        "interaction_deviation_g": metrics.get("interaction_deviation_g", 0.0),
                        "accel_mag_g": metrics.get("accel_magnitude", 0.0),
                        "gyro_mag_dps": metrics.get("gyro_magnitude", 0.0),
                        "orientation_pitch_deg": metrics.get("orientation_pitch", 0.0),
                        "orientation_roll_deg": metrics.get("orientation_roll", 0.0),
                        "synth_pitch_deg": metrics.get("synth_pitch_deg", 0.0),
                        "synth_roll_deg": metrics.get("synth_roll_deg", 0.0),
                        "interaction_active": int(bool(metrics.get("interaction_active", False))),
                        "pickup_triggered": int(bool(metrics.get("pickup_triggered", False))),
                        "impact_triggered": int(bool(metrics.get("impact_triggered", False))),
                        "raw_line": line,
                    }
                    rows.append(row)
                    writer.writerow(row)

                    now = time.perf_counter()
                    if (now - last_status_print) >= status_interval_s:
                        print_live_status(stage.name, rows[-40:], row)
                        last_status_print = now

                print(" " * 140, end="\r", flush=True)
                print(f"Completed stage: {stage.name}")

        finally:
            try:
                serial_port.close()
            except Exception:
                pass
            udp_sink.close()

    summary_text = build_summary_text(rows, config, args.baudrate)
    summary_path.write_text(summary_text + "\n", encoding="utf-8")

    print()
    print(summary_text)
    print()
    print(f"Summary written to {summary_path}")
    print(f"CSV written to {output_path}")
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return capture(args)


if __name__ == "__main__":
    raise SystemExit(main())