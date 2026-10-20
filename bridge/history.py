"""Permanent, size-capped history of the live rig, for diagnosing failures afterwards.

The bridge and synth logs are cleared whenever the rig restarts, so a dropout at a
show leaves no trace. This history is appended to across restarts and reboots and
rotated by size, so it never takes more than (backup_count + 1) x max_bytes on disk.
Every `stats_seconds` it writes one summary line, and notable events get a line each:
the receiver connecting or disappearing, sensor data stopping and returning, the
transmitter restarting or reporting no sensor samples, messages printed by the
receiver, and the object being picked up and set down.

Each line carries the wall-clock time and the seconds since boot, because a Pi
without internet keeps a wrong clock while its uptime stays right:

    2026-10-19 17:30:00.512 up 3725.4  sensor data stopped (last SEQ 1234)
    2026-10-19 17:30:09.988 up 3734.9  stats frames 0.0/s missed 0 gap - stale 10.0s | ...
"""

from __future__ import annotations

import array
import logging
import math
import shutil
import socket
import subprocess
import threading
import time
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any

from pythonosc.osc_message import OscMessage
from pythonosc.osc_message_builder import OscMessageBuilder

RECEIVER_MISSING_SECONDS = 5.0
REPEAT_LOG_SECONDS = 60.0
SYNTH_STATUS_SECONDS = 1.0


def _dbfs(value: float) -> float:
    return 20.0 * math.log10(value / 32768.0)


class _AudioLevel:
    """Peak and RMS of the default audio output, read from PipeWire in a thread."""

    COMMAND = [
        "pw-record", "-P", "{ stream.capture.sink=true }",
        "--rate", "8000", "--channels", "1", "--format", "s16", "--raw", "-",
    ]
    RETRY_SECONDS = 30.0

    def __init__(self, report) -> None:
        self._report = report
        self._lock = threading.Lock()
        self._peak = 0
        self._sumsq = 0
        self._count = 0
        self._stop = threading.Event()
        self._process: subprocess.Popen | None = None
        self._thread = threading.Thread(target=self._run, name="audio-level", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._process = subprocess.Popen(
                    self.COMMAND, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
                )
            except OSError as exc:
                self._report(f"audio level unavailable: {exc}")
                return
            carry = b""
            while not self._stop.is_set():
                chunk = self._process.stdout.read1(4096)
                if not chunk:
                    break
                data = carry + chunk
                usable = len(data) // 2 * 2
                carry = data[usable:]
                samples = array.array("h", data[:usable])
                if samples:
                    peak = max(max(samples), -min(samples))
                    sumsq = sum(s * s for s in samples)
                    with self._lock:
                        self._peak = max(self._peak, peak)
                        self._sumsq += sumsq
                        self._count += len(samples)
            self._process.wait()
            if not self._stop.is_set():
                self._report(f"audio level capture ended; retrying in {self.RETRY_SECONDS:.0f} s")
                self._stop.wait(self.RETRY_SECONDS)

    def describe(self) -> str:
        """Level since the previous call."""
        with self._lock:
            peak, sumsq, count = self._peak, self._sumsq, self._count
            self._peak = self._sumsq = self._count = 0
        if not count:
            return "no data"
        if not peak:
            return "silent"
        return f"peak {_dbfs(peak):.0f} rms {_dbfs(max(1.0, math.sqrt(sumsq / count))):.0f} dBFS"

    def close(self) -> None:
        self._stop.set()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
        self._thread.join(timeout=2.0)


class RigHistory:
    """Append rig events and periodic summaries to a rotating log file."""

    def __init__(self, path: str, config: dict[str, Any], *, synth_host: str | None = None) -> None:
        cfg = config.get("history_log", {})
        if not isinstance(cfg, dict):
            cfg = {}
        self.stats_seconds = max(1.0, float(cfg.get("stats_seconds", 10)))
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._handler = RotatingFileHandler(
            path,
            maxBytes=int(cfg.get("max_bytes", 5_000_000)),
            backupCount=int(cfg.get("backup_count", 9)),
            encoding="utf-8",
        )
        self._handler.setFormatter(logging.Formatter("%(asctime)s.%(msecs)03d %(message)s", "%Y-%m-%d %H:%M:%S"))
        self._logger = logging.getLogger(f"spacewizard.history.{id(self)}")
        self._logger.propagate = False
        self._logger.setLevel(logging.INFO)
        self._logger.addHandler(self._handler)

        self._connected: bool | None = None
        self._missing_since: float | None = None
        self._missing_reported = False
        self._stale = True
        self._stale_since: float | None = None
        self._seen_data = False
        self._last_frame: float | None = None
        self._last_seq: int | None = None
        self._tx_valid = True
        self._handled = False
        self._song = False
        self._last_poll: float | None = None
        self._next_stats: float | None = None
        self._last_message: str | None = None
        self._last_message_logged = float("-inf")
        self._message_repeats = 0
        self._reset_window(0.0)

        self._synth = None
        if synth_host:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setblocking(False)
            self._synth = (sock, (synth_host, int(cfg.get("synth_server_port", 57110))))
            self._status_request = OscMessageBuilder(address="/status").build().dgram
            self._next_status = 0.0
        self._audio = None
        if cfg.get("audio_level", True) and shutil.which("pw-record"):
            self._audio = _AudioLevel(self.event)

        self.event("bridge started" + self._boot_note())

    # ---- writing ----------------------------------------------------------
    def event(self, text: str) -> None:
        self._logger.info("up %.1f  %s", time.monotonic(), text)

    @staticmethod
    def _boot_note() -> str:
        notes = []
        try:
            notes.append("boot " + Path("/proc/sys/kernel/random/boot_id").read_text().strip()[:8])
        except OSError:
            pass
        try:
            synced = subprocess.run(
                ["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                capture_output=True, text=True, timeout=2,
            ).stdout.strip()
            if synced == "yes":
                notes.append("clock synced")
            elif synced == "no":
                notes.append("clock NOT synced, so times may be off; uptime is reliable")
        except (OSError, subprocess.SubprocessError):
            pass
        return f" ({', '.join(notes)})" if notes else ""

    def _reset_window(self, now: float) -> None:
        self._window = dict(
            start=now, frames=0, missed=0, invalid=0, malformed=0, max_gap=0.0,
            stale=0.0, handled=0.0, song=0.0, impacts=0, cpu_avg=None, cpu_peak=None, synths=None,
        )

    def _write_stats(self, now: float) -> None:
        w = self._window
        span = max(1e-6, now - w["start"])
        line = f"stats frames {w['frames'] / span:.1f}/s missed {w['missed']}"
        if w["invalid"]:
            line += f" invalid {w['invalid']}"
        if w["malformed"]:
            line += f" malformed {w['malformed']}"
        line += f" gap {w['max_gap'] * 1000:.0f}ms" if w["frames"] > 1 else " gap -"
        line += f" stale {w['stale']:.1f}s | handled {w['handled']:.1f}s song {w['song']:.1f}s"
        if w["impacts"]:
            line += f" impacts {w['impacts']}"
        if self._audio:
            line += " | audio " + self._audio.describe()
        if self._synth:
            if w["cpu_avg"] is None:
                line += " | synth no reply"
            else:
                line += f" | synth cpu {w['cpu_avg']:.0f}/{w['cpu_peak']:.0f}% {w['synths']} synths"
        self.event(line)
        self._reset_window(now)

    # ---- inputs from the bridge loop ----------------------------------------
    def poll(self, now: float, *, connected: bool, stale: bool) -> None:
        """Call once per bridge loop with the receiver and freshness state."""
        if self._last_poll is not None:
            dt = max(0.0, now - self._last_poll)
            if stale:
                self._window["stale"] += dt
            else:
                self._window["handled"] += dt if self._handled else 0.0
                self._window["song"] += dt if self._song else 0.0
        self._last_poll = now

        if connected != self._connected:
            if connected:
                note = f" after {now - self._missing_since:.1f} s" if self._missing_reported else ""
                self.event("receiver connected" + note)
                self._missing_since, self._missing_reported = None, False
            else:
                if self._connected:
                    self.event("receiver disconnected")
                self._missing_since = now
            self._connected = connected
        elif not connected and not self._missing_reported and now - self._missing_since >= RECEIVER_MISSING_SECONDS:
            self.event("no receiver on USB; retrying")
            self._missing_reported = True

        if stale != self._stale:
            if stale and self._seen_data:
                ago = now - self._last_frame if self._last_frame is not None else 0.0
                self.event(f"sensor data stopped (last frame {ago:.1f} s ago, last SEQ {self._last_seq})")
            elif not stale:
                if self._seen_data and self._stale_since is not None:
                    self.event(f"sensor data back after {now - self._stale_since:.1f} s")
                else:
                    self.event("sensor data arriving")
            self._stale_since = self._last_frame if stale else None
            self._stale = stale

        if self._synth and now >= self._next_status:
            self._poll_synth()
            self._next_status = now + SYNTH_STATUS_SECONDS

        if self._next_stats is None:
            self._next_stats = now + self.stats_seconds
            self._window["start"] = now
        elif now >= self._next_stats:
            self._write_stats(now)
            self._next_stats = now + self.stats_seconds

    def _poll_synth(self) -> None:
        sock, address = self._synth
        while True:
            try:
                data = sock.recv(65535)
            except OSError:  # nothing waiting, or the server is not running
                break
            try:
                reply = OscMessage(data)
            except Exception:
                continue
            if reply.address == "/status.reply" and len(reply.params) >= 7:
                w = self._window
                w["cpu_avg"] = max(w["cpu_avg"] or 0.0, float(reply.params[5]))
                w["cpu_peak"] = max(w["cpu_peak"] or 0.0, float(reply.params[6]))
                w["synths"] = int(reply.params[2])
        try:
            sock.sendto(self._status_request, address)
        except OSError:
            pass

    def frame(self, data: dict[str, Any], metrics: dict[str, Any], now: float) -> None:
        """Call for every accepted sensor frame."""
        w = self._window
        w["frames"] += 1
        self._seen_data = True
        if self._last_frame is not None and not self._stale:
            w["max_gap"] = max(w["max_gap"], now - self._last_frame)
        self._last_frame = now
        if data.get("SEQ") is not None:
            seq = int(data["SEQ"])
            if self._last_seq is not None:
                step = (seq - self._last_seq) & 0xFFFF
                if 0 <= seq < 50 and step > 50:
                    self.event(f"transmitter restarted (SEQ {self._last_seq} -> {seq})")
                elif 1 < step < 0x8000:
                    w["missed"] += step - 1
            self._last_seq = seq
        if not self._tx_valid:
            self.event("transmitter sensor samples valid again")
            self._tx_valid = True
        handled = bool(metrics.get("interaction_active"))
        if handled != self._handled:
            self.event("picked up" if handled else "set down")
            self._handled = handled
        self._song = bool(metrics.get("song_gate_active"))
        if metrics.get("impact_triggered"):
            w["impacts"] += 1

    def rejected_line(self, text: str, now: float) -> None:
        """Call for a serial line the parser rejected."""
        text = text.strip()
        if not text:
            return
        if text.startswith("FMT="):
            if "VALID=0" in text.split():
                self._window["invalid"] += 1
                if self._tx_valid:
                    self.event("transmitter reports no fresh sensor sample (VALID=0)")
                    self._tx_valid = False
            else:
                self._window["malformed"] += 1
            return
        text = text[:200]
        if text == self._last_message and now - self._last_message_logged < REPEAT_LOG_SECONDS:
            self._message_repeats += 1
            return
        if self._message_repeats:
            self.event(f"receiver: (previous message repeated {self._message_repeats} more times)")
        self.event(f"receiver: {text}")
        self._last_message, self._last_message_logged, self._message_repeats = text, now, 0

    def close(self) -> None:
        if self._audio:
            self._audio.close()
        if self._synth:
            self._synth[0].close()
        self.event("bridge stopped")
        self._logger.removeHandler(self._handler)
        self._handler.close()
