"""Bounded transport recovery and input freshness for the live rig."""

from __future__ import annotations

import time
from typing import Any

import serial
from pythonosc.udp_client import SimpleUDPClient

from .constants import DEFAULT_BAUD_RATE, SERIAL_TIMEOUT
from .serial_input import find_arduino_port


class RecoveringOSCClient:
    """Retry a failed destination independently, without stopping the other output."""

    def __init__(self, address: str, port: int, retry_seconds: float = 1.0):
        self._address, self._port = address, port
        self.retry_seconds = retry_seconds
        self.client = None
        self.next_retry = 0.0
        self.last_error_log = float("-inf")

    def send_message(self, address: str, value: Any) -> bool:
        now = time.monotonic()
        if now < self.next_retry:
            return False
        try:
            if self.client is None:
                self.client = SimpleUDPClient(self._address, self._port)
            self.client.send_message(address, value)
            return True
        except OSError as exc:
            self.close()
            self.next_retry = now + self.retry_seconds
            if now - self.last_error_log >= 5.0:
                print(f"OSC unavailable at {self._address}:{self._port}; retrying: {exc}")
                self.last_error_log = now
            return False

    def close(self) -> None:
        if self.client is not None:
            # python-osc's UDP client owns a socket but exposes no close method.
            sock = getattr(self.client, "_sock", None)
            if sock is not None:
                sock.close()
            self.client = None


class RecoveringSerial:
    """Keep attempting the same device path after removal or startup absence."""

    def __init__(self, path: str | None, retry_seconds: float = 1.0):
        self.path = path
        self.retry_seconds = retry_seconds
        self.connection = None
        self.next_retry = 0.0
        self.disconnected = False

    def read(self, read_latest) -> str:
        self.disconnected = False
        now = time.monotonic()
        if self.connection is None:
            if now < self.next_retry:
                time.sleep(SERIAL_TIMEOUT)
                return ""
            try:
                path = self.path or find_arduino_port()
                if not path:
                    raise serial.SerialException("No receiver detected")
                self.connection = serial.Serial(
                    port=path, baudrate=DEFAULT_BAUD_RATE, timeout=SERIAL_TIMEOUT
                )
                # Discard queued bytes from before opening/reconnecting.
                self.connection.reset_input_buffer()
                print(f"Connected to Arduino on {path}")
            except (serial.SerialException, OSError) as exc:
                self.close()
                self.next_retry = now + self.retry_seconds
                print(f"Serial unavailable on {self.path}; retrying: {exc}")
                return ""
        try:
            return read_latest(self.connection)
        except (serial.SerialException, OSError) as exc:
            self.close()
            self.disconnected = True
            self.next_retry = time.monotonic() + self.retry_seconds
            print(f"Serial connection lost; outputs off, retrying: {exc}")
            return ""

    def close(self) -> None:
        if self.connection is not None:
            try:
                self.connection.close()
            except (serial.SerialException, OSError):
                pass
            self.connection = None


class InputFreshness:
    def __init__(self, timeout_seconds: float = 0.5):
        self.timeout_seconds = timeout_seconds
        self.last_frame = None
        self.last_sequence = None

    def stale(self, now: float) -> bool:
        return self.last_frame is None or now - self.last_frame >= self.timeout_seconds

    def accept(self, data: dict, now: float) -> bool:
        sequence = data.get("SEQ")
        if sequence is not None and sequence == self.last_sequence:
            return False
        self.last_sequence = sequence
        self.last_frame = now
        return True

    def invalidate(self) -> None:
        self.last_frame = None
