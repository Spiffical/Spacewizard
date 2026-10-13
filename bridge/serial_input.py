"""Serial port discovery and line parsing helpers."""

from __future__ import annotations

import math
import serial.tools.list_ports


COMPACT_SERIAL_FORMAT_VERSION = 2.0
# Preserve the deployed FMT=2 scale and tuned thresholds. Correct physical DMP
# units must be rolled out as a coordinated firmware/config change with retuning.
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
    """Accept complete, finite sensor frames; preserve legacy engineering units."""
    try:
        parts = line.strip().split()
        data: dict[str, float | bool] = {}
        for part in parts:
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            if key in data:
                return None
            if value.lower() in ("true", "false"):
                data[key] = value.lower() == "true"
            else:
                data[key] = float(value)

        required = {"AX", "AY", "AZ", "GX", "GY", "GZ", "YAW", "PITCH", "ROLL", "IMP"}
        if not required.issubset(data) or not all(math.isfinite(v) for v in data.values()):
            return None
        version = data.get("FMT", 1.0)
        if version not in (1.0, COMPACT_SERIAL_FORMAT_VERSION):
            return None
        if data["IMP"] not in (0, 1) or data.get("VALID", 1) != 1:
            return None
        if "SEQ" in data and not (0 <= data["SEQ"] <= 65535 and float(data["SEQ"]).is_integer()):
            return None
        quaternion_keys = {"QW", "QX", "QY", "QZ"}
        if quaternion_keys.intersection(data) and not quaternion_keys.issubset(data):
            return None

        if version == COMPACT_SERIAL_FORMAT_VERSION:
            if any(not float(data[k]).is_integer() or not -32768 <= data[k] <= 32767
                   for k in required - {"IMP"} | quaternion_keys.intersection(data)):
                return None
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

        # Wide hardware limits also reject raw integer records whose FMT prefix was lost.
        if any(abs(data[k]) > 16 for k in ("AX", "AY", "AZ")):
            return None
        if any(abs(data[k]) > 2000 for k in ("GX", "GY", "GZ")):
            return None
        if any(abs(data[k]) > 360 for k in ("YAW", "PITCH", "ROLL")):
            return None
        if quaternion_keys.issubset(data):
            norm = math.sqrt(sum(data[k] ** 2 for k in quaternion_keys))
            if not 0.8 <= norm <= 1.2:
                return None
        return data
    except (ValueError, IndexError, OverflowError):
        return None
