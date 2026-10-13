"""Application entrypoint for the SpaceWizard serial-to-OSC bridge."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import threading
import time
from typing import Any

import serial

from .config import load_config
from .constants import (
    DEFAULT_MAESTRO_HOST,
    DEFAULT_MAESTRO_PORT,
)
from .display import display_compact_test_data, display_impact_test_data, display_test_data
from .metrics import ImpactState, calculate_motion_metrics, get_current_time_ms
from .osc import (
    send_osc_messages,
    send_safe_state,
    send_synth_control_messages,
    send_synth_impact_messages,
    send_synth_status_messages,
)
from .serial_input import parse_serial_line
from .public_osc import PublicOscStream
from .reliability import InputFreshness, RecoveringOSCClient, RecoveringSerial


def _read_latest_serial_line(serial_port: serial.Serial) -> str:
    """Return the newest complete serial line without blocking on future bytes."""
    raw_line = serial_port.readline()
    if not raw_line:
        return ""

    buffered_data = getattr(serial_port, "_spacewizard_pending_bytes", b"") + raw_line
    buffered_bytes = getattr(serial_port, "in_waiting", 0)
    if buffered_bytes > 0:
        buffered_data += serial_port.read(buffered_bytes)

    latest_line = b""
    pending_bytes = b""
    for line in buffered_data.splitlines(keepends=True):
        if line.endswith((b"\n", b"\r")):
            latest_line = line
        else:
            pending_bytes = line

    # A damaged stream without delimiters must not grow memory without bound.
    setattr(serial_port, "_spacewizard_pending_bytes", pending_bytes[-512:])
    if not latest_line:
        return ""

    try:
        return latest_line.decode("ascii").strip()
    except UnicodeDecodeError:
        return ""


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SpaceWizard OSC Bridge: serial motion data -> MaestroDMX OSC",
    )
    parser.add_argument("serial_port", nargs="?", help="Serial port path (auto-detected if omitted)")
    parser.add_argument("maestro_host", nargs="?", help="MaestroDMX host/IP")
    parser.add_argument("maestro_port", nargs="?", type=int, help="MaestroDMX OSC UDP port")
    parser.add_argument(
        "--config",
        default="config/bridge.json",
        help="Bridge config file (default: config/bridge.json)",
    )
    parser.add_argument(
        "--mapping-profile",
        help="Mapping profile name from the config's mapping_profiles (example: pickup)",
    )
    parser.add_argument(
        "--mapping-file",
        help="Direct mapping file path override (takes precedence over profile/default)",
    )
    parser.add_argument("-t", "--test", action="store_true", help="Test mode (no OSC send)")
    parser.add_argument("-i", "--impact-test", action="store_true", help="Impact debug mode")
    parser.add_argument(
        "-s",
        "--impact-session",
        type=float,
        default=0.0,
        help="Run timed impact session (seconds) and print report",
    )
    parser.add_argument("-p", "--session-plot", action="store_true", help="Save impact-session plot PNG")
    parser.add_argument("--live-plot", action="store_true", help="Show live interactive plot")
    parser.add_argument("--health-file", help="Optional supervisor heartbeat file")
    parser.add_argument("--no-public-osc", action="store_true", help="Do not send the public OSC stream")
    parser.add_argument("--public-osc-port", type=int, help="UDP port for the public OSC stream (default: config, 9000)")
    parser.add_argument(
        "--public-osc-target",
        action="append",
        default=[],
        metavar="HOST[:PORT]",
        help="Also send the public OSC stream to this host (repeatable)",
    )
    parser.add_argument(
        "--no-public-broadcast",
        action="store_true",
        help="Send the public OSC stream only to --public-osc-target hosts",
    )
    parser.add_argument(
        "--synth",
        action="store_true",
        help="Also send OSC control to the local SuperCollider synth patch",
    )
    parser.add_argument("--synth-host", help="SuperCollider OSC host override")
    parser.add_argument("--synth-port", type=int, help="SuperCollider OSC UDP port override")
    parser.add_argument(
        "--verbose-test",
        action="store_true",
        help="Keep the full test-mode table redraw instead of the compact CLI status line",
    )
    return parser


def _setup_live_plot(config: dict[str, Any], enabled: bool) -> tuple[bool, dict[str, Any], Any]:
    if not enabled:
        return False, {}, None

    live_data_buffer: dict[str, Any] = {
        "t": [],
        "dev": [],
        "acc": [],
        "trig": [],
        "gyro": [],
        "jerk": [],
    }

    try:
        import matplotlib.pyplot as plt
        from collections import deque
        import numpy as np

        plt.ion()
        live_data_buffer["t"] = deque(maxlen=2000)
        live_data_buffer["dev"] = deque(maxlen=2000)
        live_data_buffer["acc"] = deque(maxlen=2000)
        live_data_buffer["trig"] = deque(maxlen=2000)
        live_data_buffer["gyro"] = deque(maxlen=2000)
        live_data_buffer["jerk"] = deque(maxlen=2000)

        live_fig, live_axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        ax1, ax2 = live_axes
        (line_dev,) = ax1.plot([], [], label="Deviation (g)", color="#1f77b4")
        (line_acc,) = ax2.plot([], [], label="Accel Mag (g)", color="#2ca02c")
        thr_g = config.get("impact_detection", {}).get("threshold", 8000) / 16384.0
        rest_g_line_val = float(config.get("impact_detection", {}).get("rest_g", 0.20))
        (line_thr,) = ax1.plot([], [], "--", color="#d62728", label="Threshold (g)")
        (line_rest,) = ax1.plot([], [], ":", color="#9467bd", label="Rest g")

        ax1r = ax1.twinx()
        (line_jerk,) = ax1r.plot([], [], color="#bcbd22", alpha=0.8, label="Jerk (g/s)")
        jerk_thr = float(config.get("impact_detection", {}).get("jerk_min_gps", 2.0))
        ax1r.axhline(jerk_thr, color="#bcbd22", linestyle="--", alpha=0.5, label="Jerk Thr")
        scatter_impacts = ax1.scatter([], [], color="#ff7f0e", s=22, label="Impacts")

        ax2.axhline(1.0, color="#7f7f7f", linestyle=":", label="Baseline 1g")
        ax2r = ax2.twinx()
        (line_gyro,) = ax2r.plot([], [], label="Gyro Mag (°/s)", color="#ff7f0e", alpha=0.8)
        gyro_active = float(config.get("impact_detection", {}).get("gyro_active_dps", 120.0))
        gyro_rest = float(config.get("impact_detection", {}).get("gyro_rest_dps", 60.0))
        ax2r.axhline(gyro_active, color="#d62728", linestyle="--", label="Gyro Active (°/s)")
        ax2r.axhline(gyro_rest, color="#9467bd", linestyle=":", label="Gyro Rest (°/s)")

        ax1.legend([line_dev, line_thr, line_rest, line_jerk], ["Deviation (g)", "Threshold (g)", "Rest g", "Jerk (g/s)"], loc="upper right")
        ax2.legend([line_acc, line_gyro], ["Accel Mag (g)", "Gyro Mag (°/s)"], loc="upper right")
        ax1.set_ylabel("Deviation (g)")
        ax1r.set_ylabel("Jerk (g/s)")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Accel Mag (g)")
        ax1.grid(True, alpha=0.3)
        ax2.grid(True, alpha=0.3)

        try:
            plt.show(block=False)
        except TypeError:
            plt.show()
        live_window_sec = 10.0

        def _update_live_plot() -> None:
            if not live_data_buffer["t"]:
                return
            t = np.array(live_data_buffer["t"])
            dev = np.array(live_data_buffer["dev"])
            acc = np.array(live_data_buffer["acc"])
            trig = np.array(live_data_buffer["trig"])
            gyro = np.array(live_data_buffer["gyro"])
            jerk = np.array(live_data_buffer["jerk"])

            line_dev.set_data(t, dev)
            line_acc.set_data(t, acc)
            line_gyro.set_data(t, gyro)
            line_jerk.set_data(t, jerk)

            tmin = max(0.0, t[-1] - live_window_sec)
            tmask = t >= tmin
            tseg = t[tmask]
            if tseg.size > 0:
                line_thr.set_data(tseg, np.full_like(tseg, thr_g))
                line_rest.set_data(tseg, np.full_like(tseg, rest_g_line_val))

            if trig.any():
                xs = t[trig.astype(bool)]
                ys = dev[trig.astype(bool)]
                scatter_impacts.set_offsets(np.column_stack((xs, ys)))
            else:
                scatter_impacts.set_offsets(np.empty((0, 2)))

            ax1.set_xlim(max(0.0, t[-1] - live_window_sec), t[-1] + 0.05)
            ax2.set_xlim(ax1.get_xlim())
            ax1.set_ylim(0.0, max(1.0, float(dev.max()) * 1.1))
            ax2.set_ylim(0.0, max(1.2, float(acc.max()) * 1.1))
            ax1r.set_ylim(0.0, max(5.0, float(jerk.max()) * 1.2))
            ax2r.set_ylim(0.0, max(200.0, float(gyro.max()) * 1.2))
            live_fig.canvas.draw_idle()
            live_fig.canvas.flush_events()
            plt.pause(0.001)

        print("Live plot enabled (interactive)")
        return True, live_data_buffer, _update_live_plot

    except Exception as exc:
        print(f"Live plot disabled (matplotlib interactive backend not available): {exc}")
        return False, {}, None


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    sorted_values = sorted(values)
    index = max(0, min(len(sorted_values) - 1, int(round((pct / 100.0) * (len(sorted_values) - 1)))))
    return sorted_values[index]


def _print_session_report(
    *,
    config: dict[str, Any],
    impact_cooldown_ms: int,
    session_start_ms: int,
    sample_count: int,
    deviations_all: list[float],
    deviations_no_trig: list[float],
    impact_times: list[int],
    ts_rel_ms: list[int],
    ts_dev_g: list[float],
    ts_accel_mag_g: list[float],
    ts_trigger: list[int],
    save_plot: bool,
) -> None:
    total_ms = max(1, get_current_time_ms() - session_start_ms)
    dur_s = total_ms / 1000.0
    rate_hz = (sample_count / dur_s) if dur_s > 0 else 0.0

    max_dev = max(deviations_all) if deviations_all else 0.0
    mean_dev = (sum(deviations_all) / len(deviations_all)) if deviations_all else 0.0
    p95_dev = _percentile(deviations_all, 95)
    p95_no_trig = _percentile(deviations_no_trig, 95)
    impacts = len(impact_times)

    thr_raw = config.get("impact_detection", {}).get("threshold", 8000)
    thr_g = thr_raw / 16384.0
    suggested_thr_g = (p95_no_trig * 1.15) if deviations_no_trig else (max_dev * 1.10)
    suggested_thr_raw = int(suggested_thr_g * 16384)

    print("\n================ IMPACT SESSION REPORT ================")
    print(f"Duration: {dur_s:.2f}s | Samples: {sample_count} | Rate: {rate_hz:.1f} Hz")
    print(f"Impacts: {impacts} | Config Threshold: {thr_g:.3f}g ({thr_raw}) | Cooldown: {impact_cooldown_ms}ms")
    print(f"Deviation (g): max={max_dev:.3f} mean={mean_dev:.3f} p95={p95_dev:.3f} | no_trig_p95={p95_no_trig:.3f}")
    print(f"Suggested threshold: {suggested_thr_g:.3f}g (~{suggested_thr_raw} raw)")
    print(
        "IMPACT_REPORT "
        f"dur_s={dur_s:.2f} samples={sample_count} rate_hz={rate_hz:.1f} "
        f"impacts={impacts} max_dev_g={max_dev:.3f} mean_dev_g={mean_dev:.3f} "
        f"p95_dev_g={p95_dev:.3f} no_trig_p95_dev_g={p95_no_trig:.3f} "
        f"thr_g={thr_g:.3f} thr_raw={thr_raw} cooldown_ms={impact_cooldown_ms} "
        f"suggest_thr_g={suggested_thr_g:.3f} suggest_thr_raw={suggested_thr_raw}"
    )
    print("=======================================================\n")

    if not save_plot:
        return

    try:
        import time as _time
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        if not ts_rel_ms:
            print("No data to plot for session.")
            return

        t_sec = [x / 1000.0 for x in ts_rel_ms]
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        ax1, ax2 = axes
        ax1.plot(t_sec, ts_dev_g, label="Deviation (g)", color="#1f77b4")
        ax1.axhline(thr_g, color="#d62728", linestyle="--", label="Threshold (g)")
        if any(ts_trigger):
            trig_times = [t for t, f in zip(t_sec, ts_trigger) if f]
            trig_vals = [d for d, f in zip(ts_dev_g, ts_trigger) if f]
            ax1.scatter(trig_times, trig_vals, color="#ff7f0e", s=20, label="Impacts")
        ax1.set_ylabel("Deviation (g)")
        ax1.legend(loc="upper right")
        ax1.grid(True, alpha=0.3)

        ax2.plot(t_sec, ts_accel_mag_g, label="Accel Magnitude (g)", color="#2ca02c")
        ax2.axhline(1.0, color="#7f7f7f", linestyle=":", label="Baseline 1g")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Accel Mag (g)")
        ax2.legend(loc="upper right")
        ax2.grid(True, alpha=0.3)
        fig.suptitle("Impact Session Overview")

        out_dir = os.path.dirname(os.path.dirname(__file__))
        ts_name = _time.strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(out_dir, f"impact_session_{ts_name}.png")
        fig.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig.savefig(out_path, dpi=120)
        plt.close(fig)
        print(f"Saved impact session figure: {out_path}")
    except Exception as exc:
        print(f"Plotting skipped (install matplotlib to enable): {exc}")


def main(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)

    load_result = load_config(
        args.config,
        mapping_profile=args.mapping_profile,
        mapping_file_override=args.mapping_file,
    )
    config = load_result.config
    impact_cooldown_ms = config.get("impact_detection", {}).get("cooldown_ms", 2000)

    if load_result.config_path:
        print(f"Loaded configuration from {load_result.config_path}")
    if load_result.mapping_profile:
        print(f"Using mapping profile: {load_result.mapping_profile}")
    if load_result.mapping_file and not load_result.mapping_path:
        print(f"Selected mapping file: {load_result.mapping_file}")
    if load_result.mapping_path:
        print(f"Loaded mapping file from {load_result.mapping_path}")
    if load_result.mapping_compiled_count:
        print(f"Configured {load_result.mapping_compiled_count} OSC route mapping(s) from osc_routes.")
    for warning in load_result.warnings:
        print(f"Config warning: {warning}")

    test_mode = args.test
    impact_session_seconds = int(max(0, args.impact_session))
    impact_test_mode = args.impact_test or impact_session_seconds > 0
    session_plot = args.session_plot
    live_plot_enabled, live_data_buffer, live_update = _setup_live_plot(config, args.live_plot)

    if test_mode:
        print("🔬 TEST MODE: Data visualization only (no MaestroDMX connection)")
        print()
    elif impact_test_mode:
        print("🎯 IMPACT TEST MODE: Detailed impact detection analysis")
        print()

    serial_port_path = args.serial_port
    if test_mode:
        maestro_host = "test-mode"
        maestro_port = 0
    else:
        maestro_host = (
            args.maestro_host
            or config.get("maestro_settings", {}).get("default_host", DEFAULT_MAESTRO_HOST)
        )
        maestro_port = (
            args.maestro_port
            if args.maestro_port is not None
            else int(config.get("maestro_settings", {}).get("default_port", DEFAULT_MAESTRO_PORT))
        )

    synth_cfg = config.get("synth_settings", {})
    synth_enabled = bool(args.synth or synth_cfg.get("enabled", False))
    synth_host = args.synth_host or synth_cfg.get("host", "127.0.0.1")
    synth_port = args.synth_port if args.synth_port is not None else int(synth_cfg.get("port", 57120))
    data_processing_cfg = config.get("data_processing", {})
    output_rate_hz = float(data_processing_cfg.get("output_rate_hz", 40.0))
    output_interval_s = 1.0 / max(1.0, output_rate_hz)
    synth_control_rate_hz = float(synth_cfg.get("control_rate_hz", min(60.0, output_rate_hz)))
    synth_control_interval_s = 1.0 / max(1.0, synth_control_rate_hz)
    synth_status_rate_hz = float(synth_cfg.get("status_rate_hz", 20.0))
    synth_status_interval_s = 1.0 / max(1.0, synth_status_rate_hz)
    quiet_test_mode = bool(test_mode and synth_enabled and not args.verbose_test)

    if not serial_port_path:
        print("Receiver auto-detection enabled; waiting safely if USB is absent.")

    print("SpaceWizard OSC Bridge Starting...")
    print(f"Config loaded: {load_result.loaded}")
    print(f"Serial port: {serial_port_path or 'auto-detect'}")
    if not test_mode:
        print(f"Maestro host: {maestro_host}:{maestro_port}")
    else:
        if quiet_test_mode:
            print("Mode: Test mode with synth OSC (terminal redraw suppressed for lower latency)")
        else:
            print("Mode: Test mode (data visualization only)")
    if synth_enabled:
        print(f"SuperCollider synth: {synth_host}:{synth_port}")
    print()

    serial_port = RecoveringSerial(serial_port_path)
    osc_client = None if test_mode or impact_test_mode else RecoveringOSCClient(maestro_host, maestro_port)
    synth_client = RecoveringOSCClient(synth_host, synth_port) if synth_enabled else None
    public_cfg = config.get("public_osc", {})
    public_stream = None
    if not (
        args.no_public_osc
        or impact_test_mode
        or not (public_cfg.get("enabled", True) if isinstance(public_cfg, dict) else True)
    ):
        public_stream = PublicOscStream(
            config,
            port=args.public_osc_port,
            broadcast=False if args.no_public_broadcast else None,
            extra_targets=args.public_osc_target,
        )
    freshness = InputFreshness(float(data_processing_cfg.get("input_timeout_seconds", 0.5)))
    impact_state = ImpactState()
    session_max_deviation_g = 0.0
    last_output_time = 0.0
    last_synth_control_time = 0.0
    last_test_display_time = 0.0
    last_synth_status_time = 0.0
    last_safe_output_time = float("-inf")
    input_was_stale = True
    previous_signal_handlers = {}
    last_health_write = float("-inf")

    def stop_requested(_signum, _frame):
        raise KeyboardInterrupt

    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGTERM, getattr(signal, "SIGHUP", signal.SIGTERM)):
            if sig not in previous_signal_handlers:
                previous_signal_handlers[sig] = signal.signal(sig, stop_requested)
    if osc_client:
        print(f"Maestro OSC output configured: {maestro_host}:{maestro_port} (UDP, receipt unverified)")
    if synth_client:
        print(f"Synth OSC output configured: {synth_host}:{synth_port}")
    if public_stream:
        print(f"Public OSC stream: {public_stream.describe()}")
    print("Bridge ready; waiting for valid sensor data (outputs off).")

    session_mode = impact_session_seconds > 0
    if session_mode:
        session_start_ms = get_current_time_ms()
        deviations_all: list[float] = []
        deviations_no_trig: list[float] = []
        deviations_trig: list[float] = []
        impact_times: list[int] = []
        sample_count = 0
        ts_rel_ms: list[int] = []
        ts_dev_g: list[float] = []
        ts_accel_mag_g: list[float] = []
        ts_trigger: list[int] = []

    try:
        while True:
            try:
                now_input = time.monotonic()
                if args.health_file and now_input - last_health_write >= 1.0:
                    Path(args.health_file).touch()
                    last_health_write = now_input
                if session_mode and get_current_time_ms() - session_start_ms >= impact_session_seconds * 1000:
                    break
                if freshness.stale(now_input):
                    if not input_was_stale:
                        print("Sensor data timed out; outputs off.")
                        impact_state = ImpactState()
                        input_was_stale = True
                        last_safe_output_time = float("-inf")
                    if now_input - last_safe_output_time >= 0.2:
                        send_safe_state(config, osc_client, synth_client)
                        last_safe_output_time = now_input
                    if public_stream:
                        public_stream.send_idle(now_input)

                line = serial_port.read(_read_latest_serial_line)
                if serial_port.disconnected:
                    freshness.invalidate()
                    impact_state = ImpactState()
                    input_was_stale = True
                    send_safe_state(config, osc_client, synth_client)
                    last_safe_output_time = time.monotonic()
                    if public_stream:
                        public_stream.send_idle(force=True)
                if not line:
                    continue

                data = parse_serial_line(line)
                if not data:
                    continue
                now_input = time.monotonic()
                # The read itself may have crossed the deadline.
                if freshness.stale(now_input) and not input_was_stale:
                    impact_state = ImpactState()
                    input_was_stale = True
                if not freshness.accept(data, now_input):
                    continue
                if input_was_stale:
                    print("Fresh sensor data received.")
                    last_output_time = last_synth_control_time = 0.0
                input_was_stale = False

                metrics = calculate_motion_metrics(
                    data,
                    config,
                    impact_state,
                    impact_cooldown_ms=impact_cooldown_ms,
                )
                if not metrics:
                    continue
                if public_stream:
                    public_stream.send_frame(data, metrics, time.monotonic())

                if session_mode:
                    deviation = metrics.get("deviation_from_baseline_g", 0.0)
                    deviations_all.append(deviation)
                    sample_count += 1
                    if metrics.get("impact_triggered", False):
                        deviations_trig.append(deviation)
                        impact_times.append(get_current_time_ms())
                    else:
                        deviations_no_trig.append(deviation)

                    now_ms = get_current_time_ms()
                    ts_rel_ms.append(now_ms - session_start_ms)
                    ts_dev_g.append(deviation)
                    ts_accel_mag_g.append(metrics.get("accel_magnitude", 0.0))
                    ts_trigger.append(1 if metrics.get("impact_triggered", False) else 0)
                elif impact_test_mode:
                    session_max_deviation_g = display_impact_test_data(
                        data,
                        metrics,
                        config,
                        impact_state,
                        session_max_deviation_g,
                    )
                elif test_mode:
                    if synth_client:
                        now_synth_control = time.monotonic()
                        if (now_synth_control - last_synth_control_time) >= synth_control_interval_s:
                            send_synth_control_messages(metrics, synth_client)
                            last_synth_control_time = now_synth_control
                        send_synth_impact_messages(metrics, synth_client)
                        now_synth_status = time.monotonic()
                        if (now_synth_status - last_synth_status_time) >= synth_status_interval_s:
                            send_synth_status_messages(metrics, synth_client)
                            last_synth_status_time = now_synth_status
                    if not quiet_test_mode:
                        if args.verbose_test:
                            if synth_client:
                                display_rate_hz = float(
                                    config.get("data_processing", {}).get(
                                        "test_display_rate_hz",
                                        8,
                                    )
                                )
                                display_interval_s = 1.0 / max(1.0, display_rate_hz)
                                now_display = time.monotonic()
                                if (now_display - last_test_display_time) < display_interval_s:
                                    continue
                                last_test_display_time = now_display

                            display_test_data(data, metrics, config)
                        else:
                            display_compact_test_data(data, metrics)
                else:
                    now_output = time.monotonic()
                    if (now_output - last_output_time) >= output_interval_s:
                        send_osc_messages(metrics, config, osc_client)
                        last_output_time = now_output
                    if synth_client:
                        now_synth_control = time.monotonic()
                        if (now_synth_control - last_synth_control_time) >= synth_control_interval_s:
                            send_synth_control_messages(metrics, synth_client)
                            last_synth_control_time = now_synth_control
                        send_synth_impact_messages(metrics, synth_client)
                        now_synth_status = time.monotonic()
                        if (now_synth_status - last_synth_status_time) >= synth_status_interval_s:
                            send_synth_status_messages(metrics, synth_client)
                            last_synth_status_time = now_synth_status
                    if config.get("data_processing", {}).get("enable_debug_output", False):
                        interaction_state = "ON" if metrics.get("interaction_active", False) else "OFF"
                        print(
                            f"Accel: {metrics['accel_magnitude']:.2f}g, "
                            f"Gyro: {metrics['gyro_magnitude']:.1f}°/s, "
                            f"Yaw: {metrics['orientation_yaw']:.1f}°, "
                            f"Pitch: {metrics.get('synth_pitch_deg', metrics['orientation_pitch']):.1f}°, "
                            f"Roll: {metrics.get('synth_roll_deg', metrics['orientation_roll']):.1f}°, "
                            f"Interaction: {interaction_state}, "
                            f"Gate: {metrics.get('interaction_gate', 0.0):.2f}, "
                            f"Activity: {metrics.get('interaction_activity', 0.0):.2f}, "
                            f"Dev: {metrics.get('interaction_deviation_g', 0.0):.3f}g, "
                            f"StableΔ: {metrics.get('interaction_stable_accel_delta_g', 0.0):.3f}g/"
                            f"{metrics.get('interaction_stable_gyro_delta_dps', 0.0):.2f}°/s, "
                            f"Base: {metrics.get('interaction_baseline_g', 0.0):.3f}g, "
                            f"Pickup: {int(bool(metrics.get('pickup_triggered', False)))}"
                        )

                if live_plot_enabled and live_update:
                    try:
                        import time as _time

                        now = _time.time()
                        if not live_data_buffer["t"]:
                            live_data_buffer["_t0"] = now
                        rel = now - live_data_buffer.get("_t0", now)
                        live_data_buffer["t"].append(rel)
                        live_data_buffer["dev"].append(metrics.get("deviation_from_baseline_g", 0.0))
                        live_data_buffer["acc"].append(metrics.get("accel_magnitude", 0.0))
                        live_data_buffer["trig"].append(1 if metrics.get("impact_triggered", False) else 0)
                        live_data_buffer["gyro"].append(metrics.get("gyro_magnitude", 0.0))
                        live_data_buffer["jerk"].append(metrics.get("jerk_gps", 0.0))
                        if "_last_draw" not in live_data_buffer:
                            live_data_buffer["_last_draw"] = 0.0
                        if (now - live_data_buffer["_last_draw"]) > 0.05:
                            live_update()
                            live_data_buffer["_last_draw"] = now
                    except Exception:
                        pass

                if session_mode and (get_current_time_ms() - session_start_ms >= impact_session_seconds * 1000):
                    break

            except UnicodeDecodeError:
                continue
            except KeyboardInterrupt:
                print("\nStopping...")
                break

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        for sig, handler in previous_signal_handlers.items():
            signal.signal(sig, handler)
        # Also runs after unexpected exceptions; receiver-side watchdog covers SIGKILL.
        for client in (osc_client, synth_client):
            if client:
                client.next_retry = 0.0
        for _ in range(3):
            send_safe_state(config, osc_client, synth_client)
            time.sleep(0.03)
        if public_stream:
            public_stream.send_idle(force=True)
            public_stream.close()
        serial_port.close()
        for client in (osc_client, synth_client):
            if client:
                client.close()
        if session_mode:
            _print_session_report(
                config=config,
                impact_cooldown_ms=impact_cooldown_ms,
                session_start_ms=session_start_ms,
                sample_count=sample_count,
                deviations_all=deviations_all,
                deviations_no_trig=deviations_no_trig,
                impact_times=impact_times,
                ts_rel_ms=ts_rel_ms,
                ts_dev_g=ts_dev_g,
                ts_accel_mag_g=ts_accel_mag_g,
                ts_trigger=ts_trigger,
                save_plot=session_plot,
            )
        print("Serial connection closed.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
