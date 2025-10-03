#!/usr/bin/env python3
"""
SpaceWizard OSC Bridge - Serial to OSC Converter for MaestroDMX

This Python script reads motion data from the SpaceWizard Arduino receiver
over USB serial and converts it to OSC (Open Sound Control) messages for
MaestroDMX lighting control.

Hardware Setup:
- Arduino receiver connected via USB serial
- MaestroDMX on the same network (default: maestro.local:7672)

Serial Input Format (from Arduino):
AX=0.12 AY=0.03 AZ=0.98 GX=-10.2 GY=4.1 GZ=120.0 IMP=0

OSC Output (to MaestroDMX):
- /global/brightness: Based on overall motion intensity
- /live/1/speed: Based on gyro Z-axis rotation
- /live/1/excitement: Based on acceleration magnitude
- /triggers/strobe: Triggered by impact detection

Dependencies:
    pip install pyserial python-osc

Usage:
    python serial_to_osc_bridge.py [serial_port] [maestro_host] [maestro_port]

Default values:
- Serial port: Auto-detect (or /dev/ttyUSB0 on Linux, COM3 on Windows)
- Maestro host: maestro.local
- Maestro port: 7672
"""

import sys
import math
import json
import serial
import serial.tools.list_ports
from pythonosc.udp_client import SimpleUDPClient


# ===== CONFIGURATION =====

# Serial communication settings
DEFAULT_BAUD_RATE = 115200
SERIAL_TIMEOUT = 1.0

# Default MaestroDMX OSC settings (overridden by config file)
DEFAULT_MAESTRO_HOST = "maestro.local"
DEFAULT_MAESTRO_PORT = 7672

# Global configuration loaded from JSON file
config = None

# Impact detection cooldown tracking
last_impact_time = 0
impact_cooldown_ms = 500


# ===== GLOBAL VARIABLES =====

osc_client = None
serial_port = None


def load_config(config_path="config.json"):
    """Load configuration from JSON file."""
    global config
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
        print(f"Loaded configuration from {config_path}")
        return True
    except FileNotFoundError:
        print(f"Configuration file not found: {config_path}")
        print("Using default settings...")
        config = get_default_config()
        return False
    except json.JSONDecodeError as e:
        print(f"Error parsing configuration file: {e}")
        print("Using default settings...")
        config = get_default_config()
        return False


def get_default_config():
    """Return default configuration if config file is not found."""
    return {
        "maestro_settings": {
            "default_host": DEFAULT_MAESTRO_HOST,
            "default_port": DEFAULT_MAESTRO_PORT
        },
        "mpu6050_properties": {},
        "data_processing": {
            "smoothing_factor": 0.1,
            "deadzone_threshold": 0.05,
            "output_rate_hz": 40,
            "enable_debug_output": False
        }
    }


def find_arduino_port():
    """Auto-detect Arduino serial port."""
    ports = serial.tools.list_ports.comports()

    # Common Arduino port patterns
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
                return port.device

    return None


def clamp(value, min_val, max_val):
    """Clamp value between min and max."""
    return max(min_val, min(value, max_val))


def parse_serial_line(line):
    """Parse a serial line into motion data dictionary."""
    try:
        # Split by spaces and parse key=value pairs
        parts = line.strip().split()
        data = {}

        for part in parts:
            if '=' in part:
                key, value = part.split('=', 1)
                # Convert to float, handling both numeric and boolean values
                if value.lower() in ('true', 'false'):
                    data[key] = (value.lower() == 'true')
                else:
                    data[key] = float(value)

        return data

    except (ValueError, IndexError):
        return None


def calculate_motion_metrics(data):
    """Calculate derived motion metrics for OSC mapping."""
    if not data or not config:
        return None

    # Extract raw sensor values from the new data format
    ax = data.get('AX', 0.0)      # Accelerometer X (g-force)
    ay = data.get('AY', 0.0)      # Accelerometer Y (g-force)
    az = data.get('AZ', 0.0)      # Accelerometer Z (g-force)
    gx = data.get('GX', 0.0)      # Gyroscope X (degrees/second)
    gy = data.get('GY', 0.0)      # Gyroscope Y (degrees/second)
    gz = data.get('GZ', 0.0)      # Gyroscope Z (degrees/second)
    yaw = data.get('YAW', 0.0)    # Yaw angle (degrees)
    pitch = data.get('PITCH', 0.0) # Pitch angle (degrees)
    roll = data.get('ROLL', 0.0)  # Roll angle (degrees)
    impact_raw = data.get('IMP', False)  # Impact flag
    button = data.get('BTN', 0)   # Button state

    # Apply deadzone filtering to reduce noise
    deadzone = config.get("data_processing", {}).get("deadzone_threshold", 0.05)

    def apply_deadzone(value, threshold):
        return 0.0 if abs(value) < threshold else value

    ax = apply_deadzone(ax, deadzone)
    ay = apply_deadzone(ay, deadzone)
    az = apply_deadzone(az, deadzone)
    gx = apply_deadzone(gx, deadzone * 10)  # Gyro needs higher threshold
    gy = apply_deadzone(gy, deadzone * 10)
    gz = apply_deadzone(gz, deadzone * 10)

    # Calculate derived values
    accel_magnitude = math.sqrt(ax*ax + ay*ay + az*az)
    gyro_magnitude = math.sqrt(gx*gx + gy*gy + gz*gz)

    # Handle impact detection with cooldown
    current_time = get_current_time_ms()
    time_since_last_impact = current_time - last_impact_time

    # Check if impact should trigger (based on config or raw flag)
    impact_triggered = False
    if config.get("mpu6050_properties", {}).get("impact_detection", {}).get("enabled", False):
        impact_config = config["mpu6050_properties"]["impact_detection"]
        threshold = impact_config.get("threshold", 800)

        # Trigger if raw impact flag is set OR if acceleration magnitude exceeds threshold
        if impact_raw or accel_magnitude > (threshold / 16384.0):  # Convert threshold to g-force
            if time_since_last_impact > impact_cooldown_ms:
                impact_triggered = True
                last_impact_time = current_time

    return {
        'accelerometer_x': ax,
        'accelerometer_y': ay,
        'accelerometer_z': az,
        'gyroscope_x': gx,
        'gyroscope_y': gy,
        'gyroscope_z': gz,
        'orientation_yaw': yaw,
        'orientation_pitch': pitch,
        'orientation_roll': roll,
        'accel_magnitude': accel_magnitude,
        'gyro_magnitude': gyro_magnitude,
        'impact_triggered': impact_triggered,
        'button_state': button
    }


def get_current_time_ms():
    """Get current time in milliseconds."""
    import time
    return int(time.time() * 1000)


def send_osc_messages(metrics):
    """Send OSC messages to MaestroDMX based on configuration and motion metrics."""
    if not metrics or not config:
        return

    try:
        # Get enabled properties from configuration
        properties = config.get("mpu6050_properties", {})

        # Track which messages were sent for debug output
        messages_sent = []

        # Process each configured MPU6050 property
        for prop_name, prop_config in properties.items():
            if not prop_config.get("enabled", False):
                continue

            # Get the value from metrics
            value = metrics.get(prop_name.lower())
            if value is None:
                continue

            # Get the Maestro mappings for this property
            mappings = prop_config.get("maestro_mapping", [])
            if not mappings:
                continue

            # Process each mapping
            for mapping in mappings:
                osc_address = mapping.get("osc_address")
                if not osc_address:
                    continue

                # Apply scaling and transformation
                scaled_value = apply_mapping_transform(value, mapping)

                # Check if this is a threshold-based trigger
                if mapping.get("threshold", False):
                    threshold_min = mapping.get("scale_min", 0)
                    threshold_max = mapping.get("scale_max", 1)
                    if threshold_min <= scaled_value <= threshold_max:
                        osc_client.send_message(osc_address, True)
                        messages_sent.append(f"{osc_address}=True (threshold)")
                    else:
                        osc_client.send_message(osc_address, False)
                        messages_sent.append(f"{osc_address}=False (threshold)")
                else:
                    # Regular value mapping
                    osc_client.send_message(osc_address, scaled_value)
                    messages_sent.append(f"{osc_address}={scaled_value".3f"}")

        # Special handling for impact detection
        if metrics.get('impact_triggered', False):
            osc_client.send_message("/triggers/strobe", True)
            messages_sent.append("/triggers/strobe=True (impact)")

        # Debug output (if enabled)
        if config.get("data_processing", {}).get("enable_debug_output", False) and messages_sent:
            print(f"  → OSC: {', '.join(messages_sent[:3])}{'...' if len(messages_sent) > 3 else ''}")

    except Exception as e:
        print(f"Error sending OSC message: {e}")


def apply_mapping_transform(value, mapping):
    """Apply scaling, inversion, and clamping based on mapping configuration."""
    # Get scale parameters
    scale_min = mapping.get("scale_min", 0.0)
    scale_max = mapping.get("scale_max", 1.0)
    invert = mapping.get("invert", False)

    # Apply scaling (map input range to 0.0-1.0)
    if scale_max != scale_min:
        scaled = (value - scale_min) / (scale_max - scale_min)
    else:
        scaled = 0.0 if value < scale_min else 1.0

    # Clamp to valid range
    scaled = clamp(scaled, 0.0, 1.0)

    # Apply inversion if specified
    if invert:
        scaled = 1.0 - scaled

    return scaled


def main():
    """Main program loop."""
    global osc_client, serial_port, impact_cooldown_ms

    # Load configuration file
    config_loaded = load_config()

    # Parse command line arguments (with config file defaults)
    serial_port_path = sys.argv[1] if len(sys.argv) > 1 else find_arduino_port()
    maestro_host = sys.argv[2] if len(sys.argv) > 2 else config.get("maestro_settings", {}).get("default_host", DEFAULT_MAESTRO_HOST)
    maestro_port = int(sys.argv[3]) if len(sys.argv) > 3 else config.get("maestro_settings", {}).get("default_port", DEFAULT_MAESTRO_PORT)

    # Set impact cooldown from config
    impact_cooldown_ms = config.get("mpu6050_properties", {}).get("impact_detection", {}).get("cooldown_ms", 500)

    if not serial_port_path:
        print("Error: Could not find Arduino serial port.")
        print("Please specify the serial port as the first argument.")
        print("Example: python serial_to_osc_bridge.py /dev/ttyUSB0")
        sys.exit(1)

    print("SpaceWizard OSC Bridge Starting...")
    print(f"Config loaded: {config_loaded}")
    print(f"Serial port: {serial_port_path}")
    print(f"Maestro host: {maestro_host}:{maestro_port}")
    print()

    try:
        # Initialize OSC client
        osc_client = SimpleUDPClient(maestro_host, maestro_port)
        print(f"Connected to MaestroDMX at {maestro_host}:{maestro_port}")

        # Initialize serial connection
        serial_port = serial.Serial(
            port=serial_port_path,
            baudrate=DEFAULT_BAUD_RATE,
            timeout=SERIAL_TIMEOUT
        )
        print(f"Connected to Arduino on {serial_port_path}")
        print()

        # Skip any startup messages
        serial_port.readline()

        print("Listening for motion data...")
        print("Press Ctrl+C to stop")
        print()

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Error connecting to MaestroDMX: {e}")
        print("Make sure MaestroDMX is running and on the same network.")
        sys.exit(1)

    try:
        while True:
            try:
                # Read a line from serial
                line = serial_port.readline().decode('utf-8', errors='ignore').strip()

                if not line:
                    continue

                # Parse the serial data
                data = parse_serial_line(line)

                if data:
                    # Calculate motion metrics
                    metrics = calculate_motion_metrics(data)

                    if metrics:
                        # Send OSC messages to MaestroDMX
                        send_osc_messages(metrics)

                        # Optional: Print debug info if enabled in config
                        if config.get("data_processing", {}).get("enable_debug_output", False):
                            print(f"Accel: {metrics['accel_magnitude']:.2f}g, "
                                  f"Gyro: {metrics['gyro_magnitude']:.1f}°/s, "
                                  f"Yaw: {metrics['orientation_yaw']:.1f}°, "
                                  f"Pitch: {metrics['orientation_pitch']:.1f}°, "
                                  f"Roll: {metrics['orientation_roll']:.1f}°")

            except UnicodeDecodeError:
                # Skip garbled lines
                continue
            except KeyboardInterrupt:
                print("\nStopping...")
                break

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if serial_port and serial_port.is_open:
            serial_port.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
