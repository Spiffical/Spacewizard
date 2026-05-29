"""Serial port discovery and line parsing helpers."""

from __future__ import annotations

import serial.tools.list_ports


COMPACT_SERIAL_FORMAT_VERSION = 2.0
MPU_ACCEL_SCALE = 16384.0
MPU_GYRO_SCALE = 131.0
QUATERNION_SCALE = 16384.0


def find_arduino_port() -> str | None:
    """Auto-detect Arduino serial port, preferring USB serial over Bluetooth."""
    ports = serial.tools.list_ports.comports()

    usb_ports = []
    bluetooth_ports = []
    arduino_patterns = [
        "ttyUSB",      # Linux
        "ttyACM",      # Linux (some Arduinos)
        "usbmodem",    # macOS
        "usbserial",   # macOS
        "COM",         # Windows
    ]

    for port in ports:
        port_str = port.device.lower()
        for pattern in arduino_patterns:
            if pattern.lower() in port_str:
                if (
                    "usbserial" in port_str
                    or "usbmodem" in port_str
                    or "ttyusb" in port_str
                    or "ttyacm" in port_str
                ):
                    usb_ports.append(port.device)
                elif "bluetooth" in port_str:
                    bluetooth_ports.append(port.device)
                else:
                    usb_ports.append(port.device)

    if usb_ports:
        return usb_ports[0]
    if bluetooth_ports:
        return bluetooth_ports[0]
    return None


def parse_serial_line(line: str) -> dict[str, float | bool] | None:
    """Parse one serial line in KEY=VALUE format."""
    try:
        parts = line.strip().split()
        data: dict[str, float | bool] = {}
        for part in parts:
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            if value.lower() in ("true", "false"):
                data[key] = value.lower() == "true"
            else:
                data[key] = float(value)

        if float(data.get("FMT", 1.0)) == COMPACT_SERIAL_FORMAT_VERSION:
            for accel_key in ("AX", "AY", "AZ"):
                if accel_key in data:
                    data[accel_key] = float(data[accel_key]) / MPU_ACCEL_SCALE

            for gyro_key in ("GX", "GY", "GZ"):
                if gyro_key in data:
                    data[gyro_key] = float(data[gyro_key]) / MPU_GYRO_SCALE

            for quat_key in ("QW", "QX", "QY", "QZ"):
                if quat_key in data:
                    data[quat_key] = float(data[quat_key]) / QUATERNION_SCALE

            if "IMP" in data:
                data["IMP"] = bool(int(float(data["IMP"])))

        return data
    except (ValueError, IndexError):
        return None

