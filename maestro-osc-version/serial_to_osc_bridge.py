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
AX=0.12 AY=0.03 AZ=0.98 GX=-10.2 GY=4.1 GZ=120.0 YAW=45.0 PITCH=30.0 ROLL=-15.0 IMP=0

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
impact_cooldown_ms = 2000  # Updated to match config file
prev_deviation_above = False  # For rising-edge detection


# ===== GLOBAL VARIABLES =====

osc_client = None
serial_port = None

# Session diagnostics (impact test mode)
session_max_deviation_g = 0.0


def load_config(config_path="config.json"):
    """Load configuration from JSON file."""
    global config
    import os

    # Try multiple possible paths for the config file
    possible_paths = [
        config_path,  # Current directory
        os.path.join(os.path.dirname(__file__), config_path),  # Script directory
        os.path.join(os.getcwd(), config_path),  # Working directory
    ]

    for path in possible_paths:
        try:
            with open(path, 'r') as f:
                config = json.load(f)
            print(f"Loaded configuration from {path}")
            return True
        except FileNotFoundError:
            continue
        except json.JSONDecodeError as e:
            print(f"Error parsing configuration file {path}: {e}")
            config = get_default_config()
            return False

    print(f"Configuration file not found in any of: {[p for p in possible_paths]}")
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
    """Auto-detect Arduino serial port, preferring USB serial over Bluetooth."""
    ports = serial.tools.list_ports.comports()

    # Prioritize USB serial ports over Bluetooth
    usb_ports = []
    bluetooth_ports = []

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
                # Prioritize USB serial ports
                if "usbserial" in port_str or "usbmodem" in port_str or "ttyusb" in port_str or "ttyacm" in port_str:
                    usb_ports.append(port.device)
                elif "bluetooth" in port_str:
                    bluetooth_ports.append(port.device)
                else:
                    usb_ports.append(port.device)  # Treat other matches as USB

    # Return USB port if available, otherwise Bluetooth, otherwise None
    if usb_ports:
        return usb_ports[0]
    elif bluetooth_ports:
        return bluetooth_ports[0]

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
    global last_impact_time, prev_deviation_above
    current_time = get_current_time_ms()

    # Check if impact should trigger (based on config and magnitude)
    impact_triggered = False
    impact_cfg_all = config.get("impact_detection", config.get("mpu6050_properties", {}).get("impact_detection", {}))
    if impact_cfg_all and impact_cfg_all.get("enabled", False):
        threshold = impact_cfg_all.get("threshold", 8000)

        # More robust impact detection with multiple safeguards:
        # 1. Check if acceleration magnitude exceeds threshold (in g-force units)
        # 2. Require sustained movement (not just noise spikes)
        # 3. Ensure cooldown period has passed
        # 4. Add baseline filtering to account for sensor drift
        threshold_g = threshold / 16384.0  # Convert threshold to g-force

        # Calculate baseline (typical stationary magnitude for this sensor)
        # For a well-calibrated sensor, this should be close to 1.0g (gravity)
        baseline_magnitude = 1.0  # Assume 1g baseline for now

        # Calculate deviation from baseline (how much movement beyond normal)
        deviation_from_baseline = abs(accel_magnitude - baseline_magnitude)

        # Only trigger if:
        # - Deviation from baseline is significant (> threshold)
        # - AND cooldown period has passed (or this is the first check)
        significant_deviation = deviation_from_baseline > threshold_g

        # Rising-edge detection: only trigger when crossing from below -> above
        just_crossed_up = significant_deviation and not prev_deviation_above
        prev_deviation_above = significant_deviation

        # Allow immediate triggering on first run, then enforce cooldown
        cooldown_val = impact_cfg_all.get("cooldown_ms", impact_cooldown_ms)
        cooldown_passed = (last_impact_time == 0) or (current_time - last_impact_time > cooldown_val)

        if just_crossed_up and cooldown_passed:
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
        'deviation_from_baseline_g': abs(accel_magnitude - 1.0),
        'threshold_g': (config.get("impact_detection", config.get("mpu6050_properties", {}).get("impact_detection", {})).get("threshold", 8000) / 16384.0),
        'time_since_last_impact_ms': (current_time - last_impact_time) if last_impact_time else None
    }


def get_current_time_ms():
    """Get current time in milliseconds."""
    import time
    return int(time.time() * 1000)


def send_osc_messages(metrics):
    """Send OSC messages to MaestroDMX based on configuration and motion metrics."""
    if not metrics or not config or not osc_client:
        return

    try:
        messages_sent = []

        # 1) Prefer flat osc_mappings if present
        flat_mappings = config.get("osc_mappings", [])
        if isinstance(flat_mappings, list) and flat_mappings:
            for m in flat_mappings:
                if m.get("enabled", True) is False:
                    continue
                osc_address = m.get("osc_address")
                source = m.get("source")
                if not osc_address or not source:
                    continue

                # Resolve source value
                invert = m.get("invert", False)
                is_threshold = m.get("threshold", False)
                # Support abs: prefix
                abs_mode = False
                if isinstance(source, str) and source.startswith("abs:"):
                    abs_mode = True
                    source = source.split(":", 1)[1]
                value = metrics.get(source)
                if value is None:
                    continue
                if abs_mode and isinstance(value, (int, float)):
                    value = abs(value)

                # Boolean/threshold-only mappings
                if is_threshold:
                    bool_value = bool(value)
                    osc_client.send_message(osc_address, bool_value)
                    messages_sent.append(f"{osc_address}={'True' if bool_value else 'False'} (flat)")
                    continue

                # Numeric scaling
                scaled_value = value
                if isinstance(value, (int, float)):
                    scale_min = m.get("scale_min", 0.0)
                    scale_max = m.get("scale_max", 1.0)
                    # Map to 0-1 range based on provided scale
                    if scale_max != scale_min:
                        scaled_value = (value - scale_min) / (scale_max - scale_min)
                    else:
                        scaled_value = 0.0 if value < scale_min else 1.0
                    # Clamp
                    scaled_value = clamp(scaled_value, 0.0, 1.0)
                    if invert:
                        scaled_value = 1.0 - scaled_value
                osc_client.send_message(osc_address, scaled_value)
                messages_sent.append(f"{osc_address}={scaled_value:.3f} (flat)")
        else:
            # 2) Fallback to old property-based mappings
            properties = config.get("mpu6050_properties", {})
            for prop_name, prop_config in properties.items():
                if not prop_config.get("enabled", False):
                    continue
                value = metrics.get(prop_name.lower())
                if value is None:
                    continue
                mappings = prop_config.get("maestro_mapping", [])
                if not mappings:
                    continue
                for mapping in mappings:
                    osc_address = mapping.get("osc_address")
                    if not osc_address:
                        continue
                    scaled_value = apply_mapping_transform(value, mapping)
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
                        osc_client.send_message(osc_address, scaled_value)
                        messages_sent.append(f"{osc_address}={scaled_value:.3f}")

        # Special handling for impact detection
        if metrics.get('impact_triggered', False):
            osc_client.send_message("/triggers/strobe", True)
            messages_sent.append("/triggers/strobe=True (impact)")

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


def display_test_data(data, metrics):
    """Display motion data in a nice tabular format for test mode."""
    import time

    # Clear screen and move cursor to top (works on most terminals)
    print("\033[2J\033[H", end="")

    print("🔬 SpaceWizard Motion Data Test Mode")
    print("=" * 70)
    print(f"Timestamp: {time.strftime('%H:%M:%S')}")
    print()

    # Raw sensor data table
    print("📊 RAW SENSOR DATA:")
    print("-" * 50)
    print(f"{'Accelerometer (g):':>20} | {data.get('AX', 0):>8.3f} | {data.get('AY', 0):>8.3f} | {data.get('AZ', 0):>8.3f}")
    print(f"{'Gyroscope (°/s):':>20} | {data.get('GX', 0):>8.1f} | {data.get('GY', 0):>8.1f} | {data.get('GZ', 0):>8.1f}")
    print()

    # Orientation data table
    print("🧭 ORIENTATION:")
    print("-" * 50)
    print(f"{'Yaw (degrees):':>20} | {data.get('YAW', 0):>8.1f}°")
    print(f"{'Pitch (degrees):':>20} | {data.get('PITCH', 0):>8.1f}°")
    print(f"{'Roll (degrees):':>20} | {data.get('ROLL', 0):>8.1f}°")
    print()

    # Calculated metrics table
    print("⚡ CALCULATED METRICS:")
    print("-" * 50)
    print(f"{'Accel Magnitude (g):':>20} | {metrics.get('accel_magnitude', 0):>8.3f}")
    print(f"{'Gyro Magnitude (°/s):':>20} | {metrics.get('gyro_magnitude', 0):>8.1f}")
    print()

    # Impact detection
    impact_status = "🚨 IMPACT DETECTED!" if metrics.get('impact_triggered', False) else "✅ No impact"
    print(f"{'Impact Status:':>20} | {impact_status}")
    print()

    # OSC preview (what would be sent to MaestroDMX)
    print("🎛️  OSC MESSAGES (Preview):")
    print("-" * 50)

    # Get enabled properties and show what OSC messages would be sent
    properties = config.get("mpu6050_properties", {})
    messages = []

    for prop_name, prop_config in properties.items():
        if prop_config.get("enabled", False):
            value = metrics.get(prop_name.lower())
            if value is not None:
                mappings = prop_config.get("maestro_mapping", [])
                for mapping in mappings:
                    osc_address = mapping.get("osc_address")
                    if osc_address:
                        scaled_value = apply_mapping_transform(value, mapping)
                        if mapping.get("threshold", False):
                            threshold_min = mapping.get("scale_min", 0)
                            threshold_max = mapping.get("scale_max", 1)
                            if threshold_min <= scaled_value <= threshold_max:
                                messages.append(f"{osc_address}=True")
                            else:
                                messages.append(f"{osc_address}=False")
                        else:
                            messages.append(f"{osc_address}={scaled_value:.3f}")

    if messages:
        for msg in messages[:5]:  # Show first 5 messages
            print(f"  → {msg}")
        if len(messages) > 5:
            print(f"  ... and {len(messages) - 5} more messages")
    else:
        print("  No OSC messages configured")

    print()
    print("=" * 70)
    print("💡 Tip: Move your SpaceWizard device to see real-time motion data!")
    print("   Press Ctrl+C to exit test mode")


def display_impact_test_data(data, metrics):
    """Display detailed impact detection analysis for debugging."""
    import time

    # Clear screen and move cursor to top (works on most terminals)
    print("\033[2J\033[H", end="")

    print("🎯 SpaceWizard Impact Detection Test Mode")
    print("=" * 80)
    print(f"Timestamp: {time.strftime('%H:%M:%S')}")
    print()

    # Raw accelerometer data with detailed analysis
    ax = data.get('AX', 0)
    ay = data.get('AY', 0)
    az = data.get('AZ', 0)

    print("📊 ACCELEROMETER ANALYSIS:")
    print("-" * 60)
    # Detect if incoming values are raw counts or already in g's
    is_raw_counts = max(abs(ax), abs(ay), abs(az)) > 4.0
    ax_g = (ax / 16384.0) if is_raw_counts else ax
    ay_g = (ay / 16384.0) if is_raw_counts else ay
    az_g = (az / 16384.0) if is_raw_counts else az
    print(f"{'Raw Values:':>15} | X: {ax:>8.3f} | Y: {ay:>8.3f} | Z: {az:>8.3f}")
    print(f"{'As g-force:':>15} | X: {ax_g:>8.3f} | Y: {ay_g:>8.3f} | Z: {az_g:>8.3f}")

    accel_magnitude = metrics.get('accel_magnitude', 0)
    baseline_magnitude = 1.0  # Expected gravity reading
    deviation_from_baseline = abs(accel_magnitude - baseline_magnitude)

    print(f"{'Magnitude:':>15} | Total: {accel_magnitude:>8.3f}g")
    print(f"{'Baseline (1g):':>15} | Expected: {baseline_magnitude:>8.3f}g")
    print(f"{'Deviation:':>15} | Difference: {deviation_from_baseline:>8.3f}g")
    print()

    # Impact detection logic breakdown
    print("🎯 IMPACT DETECTION ANALYSIS:")
    print("-" * 60)

    # Get current config values
    impact_cfg_all = config.get("impact_detection", config.get("mpu6050_properties", {}).get("impact_detection", {}))
    threshold = impact_cfg_all.get("threshold", 8000)
    threshold_g = threshold / 16384.0

    # Current time and last impact time
    current_time = get_current_time_ms()
    time_since_last_impact = (current_time - last_impact_time) if last_impact_time else None
    cooldown_remaining = (impact_cfg_all.get("cooldown_ms", impact_cooldown_ms) - time_since_last_impact) if (time_since_last_impact is not None) else None
    if cooldown_remaining is not None and cooldown_remaining < 0:
        cooldown_remaining = 0

    print(f"{'Threshold:':>15} | Raw: {threshold:>6d} | In g: {threshold_g:>6.3f}g")
    if cooldown_remaining is not None:
        print(f"{'Cooldown:':>15} | {cooldown_remaining:>6d}ms remaining")
    else:
        print(f"{'Cooldown:':>15} | {'N/A':>6}")

    # Last impact time display
    if time_since_last_impact is not None:
        print(f"{'Last Impact:':>15} | {time_since_last_impact:>6d}ms ago")
    else:
        print(f"{'Last Impact:':>15} | {'never':>6}")
    print()

    # Decision logic
    print("🔍 DECISION LOGIC:")
    print("-" * 60)

    significant_deviation = deviation_from_baseline > threshold_g
    cooldown_passed = (time_since_last_impact is None) or (time_since_last_impact > impact_cfg_all.get("cooldown_ms", impact_cooldown_ms))
    impact_triggered = significant_deviation and cooldown_passed

    print(f"{'Deviation > Threshold:':>25} | {str(significant_deviation):>5} (Deviation: {deviation_from_baseline:.3f}g > {threshold_g:.3f}g)")
    if time_since_last_impact is not None:
        print(f"{'Cooldown Passed:':>25} | {str(cooldown_passed):>5} (Time: {time_since_last_impact}ms > {impact_cfg_all.get('cooldown_ms', impact_cooldown_ms)}ms)")
    else:
        print(f"{'Cooldown Passed:':>25} | {str(cooldown_passed):>5} (No prior impacts)")
    print(f"{'Impact Triggered:':>25} | {'🚨 YES' if impact_triggered else '✅ NO':>5}")
    print()

    # Final result
    if impact_triggered:
        print("🚨 IMPACT DETECTED! OSC message would be sent to MaestroDMX")
        print("   → /triggers/strobe=True")
    else:
        print("✅ No impact detected - sensor readings are within normal range")

    # Session diagnostics and copyable summary line
    global session_max_deviation_g
    if deviation_from_baseline > session_max_deviation_g:
        session_max_deviation_g = deviation_from_baseline

    recommended_threshold_g = session_max_deviation_g * 1.10  # 10% headroom
    recommended_threshold_raw = int(recommended_threshold_g * 16384)

    print()
    print("-" * 80)
    print(f"Session Max Deviation: {session_max_deviation_g:.3f}g | Suggested threshold: {recommended_threshold_g:.3f}g (~{recommended_threshold_raw} raw)")
    # Copy/paste-friendly one-line summary for tuning
    summary_parts = [
        f"ts_ms={current_time}",
        f"ax_g={ax_g:.3f}", f"ay_g={ay_g:.3f}", f"az_g={az_g:.3f}",
        f"mag_g={accel_magnitude:.3f}", f"dev_g={deviation_from_baseline:.3f}",
        f"thr_g={threshold_g:.3f}", f"thr_raw={threshold}",
        f"since_ms={(time_since_last_impact if time_since_last_impact is not None else -1)}",
        f"rem_ms={(cooldown_remaining if cooldown_remaining is not None else -1)}",
        f"cooldown_ms={impact_cfg_all.get('cooldown_ms', impact_cooldown_ms)}",
        f"sig={int(significant_deviation)}", f"cool={int(cooldown_passed)}", f"trig={int(impact_triggered)}"
    ]
    print("IMPACT_SUMMARY " + " ".join(summary_parts))

    print()
    print("=" * 80)
    print("💡 DEBUGGING TIPS:")
    print("   • Green checkmarks = condition met")
    print("   • Red X marks = condition not met")
    print("   • Adjust threshold in config.json if too sensitive/insensitive")
    print("   • Check if sensor is perfectly level (affects baseline)")
    print("   • Press Ctrl+C to exit impact test mode")


def main():
    """Main program loop."""
    global osc_client, serial_port, impact_cooldown_ms

    # Load configuration file
    config_loaded = load_config()

    # Check for test modes
    test_mode = "--test" in sys.argv or "-t" in sys.argv
    impact_test_mode = "--impact-test" in sys.argv or "-i" in sys.argv
    # Timed impact session (seconds). Use: --impact-session 5 or --impact-session=5 or -s 5
    def _get_arg_seconds(short_flag, long_flag):
        for i, arg in enumerate(sys.argv):
            if arg == short_flag and i + 1 < len(sys.argv):
                try:
                    return int(float(sys.argv[i + 1]))
                except Exception:
                    return 0
            if arg.startswith(long_flag + "="):
                try:
                    return int(float(arg.split("=", 1)[1]))
                except Exception:
                    return 0
            if arg == long_flag and i + 1 < len(sys.argv):
                try:
                    return int(float(sys.argv[i + 1]))
                except Exception:
                    return 0
        return 0
    impact_session_seconds = _get_arg_seconds("-s", "--impact-session")
    if impact_session_seconds > 0:
        impact_test_mode = True

    if test_mode:
        print("🔬 TEST MODE: Data visualization only (no MaestroDMX connection)")
        print()
    elif impact_test_mode:
        print("🎯 IMPACT TEST MODE: Detailed impact detection analysis")
        print()

    # Parse command line arguments
    serial_port_path = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") and not sys.argv[1].startswith("-") else find_arduino_port()

    if test_mode:
        maestro_host = "test-mode"  # Not used in test mode
        maestro_port = 0           # Not used in test mode
    else:
        maestro_host = sys.argv[2] if len(sys.argv) > 2 else config.get("maestro_settings", {}).get("default_host", DEFAULT_MAESTRO_HOST)
        maestro_port = int(sys.argv[3]) if len(sys.argv) > 3 else config.get("maestro_settings", {}).get("default_port", DEFAULT_MAESTRO_PORT)

    # Set impact cooldown from config
    impact_cooldown_ms = config.get("impact_detection", {}).get("cooldown_ms", 2000)

    if not serial_port_path:
        print("Error: Could not find Arduino serial port.")
        print("Please specify the serial port as the first argument.")
        print("Example: python serial_to_osc_bridge.py /dev/ttyUSB0")
        print("Or run in test mode: python serial_to_osc_bridge.py --test")
        sys.exit(1)

    print("SpaceWizard OSC Bridge Starting...")
    print(f"Config loaded: {config_loaded}")
    print(f"Serial port: {serial_port_path}")
    if not test_mode:
        print(f"Maestro host: {maestro_host}:{maestro_port}")
    else:
        print("Mode: Test mode (data visualization only)")
    print()

    try:
        # Initialize serial connection first (needed for both modes)
        serial_port = serial.Serial(
            port=serial_port_path,
            baudrate=DEFAULT_BAUD_RATE,
            timeout=SERIAL_TIMEOUT
        )
        print(f"Connected to Arduino on {serial_port_path}")
        print()

        # Initialize OSC client only if not in test mode
        if not test_mode:
            osc_client = SimpleUDPClient(maestro_host, maestro_port)
            print(f"Connected to MaestroDMX at {maestro_host}:{maestro_port}")
        else:
            osc_client = None
            print("Test mode: No MaestroDMX connection needed")

        print()

        # Skip any startup messages
        serial_port.readline()

        if impact_test_mode:
            print("🎯 IMPACT TEST MODE: Analyzing impact detection logic...")
            print("Press Ctrl+C to stop")
            print("=" * 80)
            if impact_session_seconds > 0:
                print(f"⏱️ IMPACT SESSION: {impact_session_seconds}s (collecting stats, no live table)")
        elif test_mode:
            print("🔬 TEST MODE: Visualizing motion data...")
            print("Press Ctrl+C to stop")
            print("=" * 70)
        else:
            print("Listening for motion data...")
            print("Press Ctrl+C to stop")
        print()

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        if test_mode:
            print("Cannot run test mode without a serial connection.")
            print("Please connect your Arduino or specify the correct serial port.")
        sys.exit(1)
    except Exception as e:
        if not test_mode:
            print(f"Error connecting to MaestroDMX: {e}")
            print("Make sure MaestroDMX is running and on the same network.")
            print("Tip: Run 'python serial_to_osc_bridge.py --test' to test without MaestroDMX")
        else:
            print(f"Error in test mode: {e}")
        sys.exit(1)

    try:
        # Impact session accumulators
        session_mode = ('impact_session_seconds' in locals()) and (impact_session_seconds > 0)
        if session_mode:
            session_start_ms = get_current_time_ms()
            deviations_all = []
            deviations_no_trig = []
            deviations_trig = []
            impact_times = []
            sample_count = 0
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
                        if session_mode:
                            # Collect stats silently during timed session
                            dev = metrics.get('deviation_from_baseline_g', 0.0)
                            deviations_all.append(dev)
                            sample_count += 1
                            if metrics.get('impact_triggered', False):
                                deviations_trig.append(dev)
                                impact_times.append(get_current_time_ms())
                            else:
                                deviations_no_trig.append(dev)
                        elif impact_test_mode:
                            # Impact test mode: Detailed impact detection analysis
                            display_impact_test_data(data, metrics)
                        elif test_mode:
                            # Test mode: Display data in a nice table format
                            display_test_data(data, metrics)
                        else:
                            # Normal mode: Send OSC messages to MaestroDMX
                            send_osc_messages(metrics)

                            # Optional: Print debug info if enabled in config
                            if config.get("data_processing", {}).get("enable_debug_output", False):
                                print(f"Accel: {metrics['accel_magnitude']:.2f}g, "
                                      f"Gyro: {metrics['gyro_magnitude']:.1f}°/s, "
                                      f"Yaw: {metrics['orientation_yaw']:.1f}°, "
                                      f"Pitch: {metrics['orientation_pitch']:.1f}°, "
                                      f"Roll: {metrics['orientation_roll']:.1f}°")

                # Stop after session window
                if session_mode and (get_current_time_ms() - session_start_ms >= impact_session_seconds * 1000):
                    break

            except UnicodeDecodeError:
                # Skip garbled lines
                continue
            except KeyboardInterrupt:
                print("\nStopping...")
                break

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Print impact session summary if requested
        if ('session_mode' in locals()) and session_mode:
            total_ms = max(1, get_current_time_ms() - session_start_ms)
            dur_s = total_ms / 1000.0
            rate_hz = (sample_count / dur_s) if ('sample_count' in locals() and dur_s > 0) else 0.0
            def _percentile(values, pct):
                if not values:
                    return 0.0
                vs = sorted(values)
                k = max(0, min(len(vs) - 1, int(round((pct / 100.0) * (len(vs) - 1)))))
                return vs[k]
            max_dev = max(deviations_all) if deviations_all else 0.0
            mean_dev = (sum(deviations_all) / len(deviations_all)) if deviations_all else 0.0
            p95_dev = _percentile(deviations_all, 95)
            p95_no_trig = _percentile(deviations_no_trig, 95)
            impacts = len(impact_times) if ('impact_times' in locals()) else 0
            # Current configured threshold
            _thr_raw = config.get("impact_detection", {}).get("threshold", 8000)
            _thr_g = _thr_raw / 16384.0
            # Suggest threshold: 15% above 95th percentile of non-trigger deviations (or 10% above max if none)
            suggested_thr_g = (p95_no_trig * 1.15) if deviations_no_trig else (max_dev * 1.10)
            suggested_thr_raw = int(suggested_thr_g * 16384)
            print("\n================ IMPACT SESSION REPORT ================")
            print(f"Duration: {dur_s:.2f}s | Samples: {sample_count} | Rate: {rate_hz:.1f} Hz")
            print(f"Impacts: {impacts} | Config Threshold: {_thr_g:.3f}g ({_thr_raw}) | Cooldown: {impact_cooldown_ms}ms")
            print(f"Deviation (g): max={max_dev:.3f} mean={mean_dev:.3f} p95={p95_dev:.3f} | no_trig_p95={p95_no_trig:.3f}")
            print(f"Suggested threshold: {suggested_thr_g:.3f}g (~{suggested_thr_raw} raw)")
            print(
                "IMPACT_REPORT "
                f"dur_s={dur_s:.2f} samples={sample_count} rate_hz={rate_hz:.1f} "
                f"impacts={impacts} max_dev_g={max_dev:.3f} mean_dev_g={mean_dev:.3f} "
                f"p95_dev_g={p95_dev:.3f} no_trig_p95_dev_g={p95_no_trig:.3f} "
                f"thr_g={_thr_g:.3f} thr_raw={_thr_raw} cooldown_ms={impact_cooldown_ms} "
                f"suggest_thr_g={suggested_thr_g:.3f} suggest_thr_raw={suggested_thr_raw}"
            )
            print("=======================================================\n")
        if serial_port and serial_port.is_open:
            serial_port.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
