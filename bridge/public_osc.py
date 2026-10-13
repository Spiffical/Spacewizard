"""Public OSC stream: the motion sensor's data, broadcast on the local network for anyone.

Every sensor frame becomes one OSC bundle (rate limited) sent by UDP broadcast to
`port` on every IPv4 network this machine is on, plus any explicit host:port
targets. Receivers just listen on the port; there is nothing to connect to.

Addresses (default prefix /spacewizard/sensor):
    /valid        i    1 while fresh sensor data arrives, 0 when it stops
    /active       i    1 while the object is being handled (lights and sound on)
    /activity     f    instantaneous movement, 0..1
    /energy       f    averaged movement, 0..1 (drives the song mode)
    /orientation  fff  object pitch -90..90, roll -180..180, yaw 0..360 (degrees)
    /quaternion   ffff sensor orientation w x y z
    /accel        fff  sensor acceleration in g (includes gravity)
    /gyro         fff  sensor rotation rate in degrees per second
    /pickup       i    event: the object was just picked up
    /impact       f    event: a knock or drop, jerk in g per second
"""

from __future__ import annotations

import json
import math
import socket
import subprocess
import time
from typing import Any

from pythonosc.osc_bundle_builder import IMMEDIATELY, OscBundleBuilder
from pythonosc.osc_message_builder import OscMessageBuilder

# The bridge keeps legacy internal units (acceleration / 16384, rotation / 131) that its
# motion thresholds were tuned in. The transmitter's DMP acceleration is 8192 counts per
# g and its gyro runs at +/-2000 deg/s (16.4 counts per deg/s), so convert to real units.
ACCEL_TO_G = 16384.0 / 8192.0
GYRO_TO_DPS = 131.0 / 16.4

BROADCAST_REFRESH_SECONDS = 10.0
ERROR_LOG_SECONDS = 30.0


def parse_target(target: str, default_port: int) -> tuple[str, int]:
    """Parse "host" or "host:port"."""
    head, _, tail = str(target).strip().rpartition(":")
    if not head:
        return tail, default_port
    return head, int(tail)


def broadcast_addresses() -> list[str]:
    """Broadcast address of every IPv4 network (Linux `ip`), else the limited broadcast."""
    try:
        result = subprocess.run(
            ["ip", "-4", "-j", "addr", "show"],
            capture_output=True, text=True, timeout=2, check=True,
        )
        found = sorted({
            info["broadcast"]
            for interface in json.loads(result.stdout)
            for info in interface.get("addr_info", [])
            if info.get("broadcast")
        })
        if found:
            return found
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError):
        pass
    return ["255.255.255.255"]


def _wrap_degrees(value: float) -> float:
    return ((value + 180.0) % 360.0) - 180.0


class PublicOscStream:
    """Rate-limited OSC bundles to broadcast addresses and explicit targets."""

    def __init__(
        self,
        config: dict[str, Any],
        *,
        port: int | None = None,
        broadcast: bool | None = None,
        extra_targets: list[str] | None = None,
    ) -> None:
        cfg = config.get("public_osc", {})
        if not isinstance(cfg, dict):
            cfg = {}
        self.port = int(port or cfg.get("port", 9000))
        self.broadcast = bool(cfg.get("broadcast", True) if broadcast is None else broadcast)
        self.prefix = "/" + str(cfg.get("address_prefix", "/spacewizard/sensor")).strip("/")
        self.interval = 1.0 / max(1.0, float(cfg.get("rate_hz", 30)))
        self.idle_interval = 1.0 / max(0.2, float(cfg.get("idle_rate_hz", 2)))
        self.targets = [
            parse_target(t, self.port) for t in list(cfg.get("targets", [])) + list(extra_targets or [])
        ]
        self._sock: socket.socket | None = None
        self._broadcast_addrs: list[str] = []
        self._next_refresh = 0.0
        self._last_frame = float("-inf")
        self._last_idle = float("-inf")
        self._last_error_log = float("-inf")
        self._errors = 0

    def describe(self) -> str:
        parts = []
        if self.broadcast:
            parts.append("broadcast to " + ", ".join(self._destinations_now(time.monotonic(), hosts_only=True)))
        parts += [f"{host}:{port}" for host, port in self.targets]
        return f"UDP port {self.port}, {self.prefix}/*: " + ("; ".join(parts) or "no destinations")

    def _destinations_now(self, now: float, hosts_only: bool = False) -> list[Any]:
        if self.broadcast and now >= self._next_refresh:
            self._broadcast_addrs = broadcast_addresses()
            self._next_refresh = now + BROADCAST_REFRESH_SECONDS
        if hosts_only:
            return list(self._broadcast_addrs) if self.broadcast else []
        destinations = [(address, self.port) for address in self._broadcast_addrs] if self.broadcast else []
        return destinations + [t for t in self.targets if t not in destinations]

    def _send(self, dgram: bytes, now: float) -> None:
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self._sock.setblocking(False)
        for destination in self._destinations_now(now):
            try:
                self._sock.sendto(dgram, destination)
            except OSError as exc:
                self._errors += 1
                if now - self._last_error_log >= ERROR_LOG_SECONDS:
                    print(f"Public OSC send to {destination[0]}:{destination[1]} failed ({self._errors} so far): {exc}")
                    self._last_error_log = now

    def _bundle(self, messages: list[tuple[str, list[Any]]]) -> bytes:
        bundle = OscBundleBuilder(IMMEDIATELY)
        for name, values in messages:
            message = OscMessageBuilder(address=f"{self.prefix}/{name}")
            for value in values:
                message.add_arg(value, "i" if isinstance(value, int) else "f")
            bundle.add_content(message.build())
        return bundle.build().dgram

    def send_frame(self, data: dict[str, Any], metrics: dict[str, Any], now: float | None = None) -> bool:
        """Send one frame if the rate limit allows; returns whether it was sent."""
        now = time.monotonic() if now is None else now
        # Events go out immediately; only the continuous frame is rate limited.
        if metrics.get("pickup_triggered"):
            self._send(self._bundle([("pickup", [1])]), now)
        if metrics.get("impact_triggered"):
            jerk = float(metrics.get("jerk_gps", 0.0)) * ACCEL_TO_G
            self._send(self._bundle([("impact", [jerk if math.isfinite(jerk) else 0.0])]), now)
        if now - self._last_frame < self.interval:
            return False
        self._last_frame = now
        f = lambda key, scale=1.0: float(data.get(key, 0.0)) * scale  # noqa: E731
        messages: list[tuple[str, list[Any]]] = [
            ("valid", [1]),
            ("active", [1 if metrics.get("interaction_active") else 0]),
            ("activity", [float(metrics.get("interaction_activity", 0.0))]),
            ("energy", [float(metrics.get("song_energy", 0.0))]),
            ("orientation", [
                float(metrics.get("synth_pitch_deg", 0.0)),
                _wrap_degrees(float(metrics.get("synth_roll_deg", 0.0))),
                (float(metrics.get("synth_yaw_norm", 0.0)) % 1.0) * 360.0,
            ]),
        ]
        if all(key in data for key in ("QW", "QX", "QY", "QZ")):
            messages.append(("quaternion", [f("QW"), f("QX"), f("QY"), f("QZ")]))
        messages.append(("accel", [f("AX", ACCEL_TO_G), f("AY", ACCEL_TO_G), f("AZ", ACCEL_TO_G)]))
        messages.append(("gyro", [f("GX", GYRO_TO_DPS), f("GY", GYRO_TO_DPS), f("GZ", GYRO_TO_DPS)]))
        self._send(self._bundle(messages), now)
        return True

    def send_idle(self, now: float | None = None, force: bool = False) -> bool:
        """While sensor data is missing, say so at a low rate."""
        now = time.monotonic() if now is None else now
        if not force and now - self._last_idle < self.idle_interval:
            return False
        self._last_idle = now
        self._send(self._bundle([("valid", [0]), ("active", [0]), ("activity", [0.0]), ("energy", [0.0])]), now)
        return True

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None
