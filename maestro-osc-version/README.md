# SpaceWizard Maestro OSC Version

This version of SpaceWizard controls **MaestroDMX** lighting systems using **OSC (Open Sound Control)** instead of directly controlling NeoPixel LEDs.

## Overview

The system consists of three parts:
1. **Transmitter Arduino** - MPU6050 motion sensor with nRF24L01 wireless transmitter
2. **Receiver Arduino** - Receives wireless data and outputs serial for Python processing
3. **Python OSC Bridge** - Converts serial data to OSC messages for MaestroDMX

## Hardware Requirements

### Transmitter (Unchanged from LED version)
- Arduino Nano/UNO
- MPU6050 6-axis accelerometer/gyroscope
- nRF24L01 radio transceiver module
- Push button for mode switching

### Receiver (Simplified)
- Arduino Nano/UNO
- nRF24L01 radio transceiver module
- USB connection to laptop running Python script

### Additional Requirements
- Laptop/computer with Python 3.6+
- MaestroDMX software running on the same network
- Network connection between laptop and MaestroDMX

## Software Architecture

```
[Transmitter Arduino] → [nRF24L01 Wireless] → [Receiver Arduino] → [USB Serial] → [Python OSC Bridge] → [OSC/UDP] → [MaestroDMX]
```

## OSC Message Mapping

The system uses a **configuration file** (`config.json`) to define which MPU6050 sensor data maps to which MaestroDMX OSC addresses. This provides complete flexibility in controlling lighting parameters.

### Default Configuration

| MPU6050 Data | OSC Address | Description |
|--------------|-------------|-------------|
| Accelerometer X | `/live/1/brightness` | Left-right tilt controls brightness |
| Accelerometer Y | `/live/1/speed` | Forward-back tilt controls animation speed |
| Accelerometer Z | `/global/brightness` | Up-down motion controls global brightness |
| Accelerometer Z | `/triggers/strobe` | Sharp upward movements trigger strobe |
| Gyroscope X | `/live/1/energy` | Roll rotation controls energy parameter |
| Gyroscope Y | `/live/1/excitement` | Pitch rotation controls excitement |
| Gyroscope Z | `/live/1/motion/speed` | Yaw rotation controls motion speed |
| Orientation Yaw | `/live/1/shape` | Rotation around Z-axis controls pattern shape |
| Orientation Pitch | `/live/1/variance` | Up-down tilt controls pattern variance |
| Orientation Roll | `/live/1/decay` | Left-right tilt controls pattern decay |
| Impact Detection | `/triggers/strobe` | Sudden movements trigger strobe effects |

### Configuration File

The `config.json` file allows you to:
- **Enable/disable** specific sensor data sources
- **Map any sensor data** to any OSC address
- **Set scaling ranges** for proper value mapping
- **Configure thresholds** for trigger effects
- **Customize sensitivity** and deadzone settings

## Installation & Setup

### 1. Install Python Dependencies

```bash
pip install -r requirements.txt
```

Or manually:
```bash
pip install pyserial python-osc
```

### 2. Configure MaestroDMX

1. Ensure MaestroDMX is running
2. Note the network settings (default: `maestro.local:7672`)
3. Verify OSC is enabled in MaestroDMX settings

### 3. Upload Arduino Code

1. Upload transmitter code (unchanged from LED version) to transmitter Arduino
2. Upload receiver code (`Spacewizard2point0_osc_receiver.ino`) to receiver Arduino

### 4. Run the OSC Bridge

```bash
# Auto-detect serial port and use default Maestro settings
python serial_to_osc_bridge.py

# Or specify custom settings
python serial_to_osc_bridge.py /dev/ttyUSB0 maestro.local 7672

# Test mode - visualize motion data without MaestroDMX connection
python serial_to_osc_bridge.py --test
```

## Test Mode

The OSC bridge includes a **test mode** that allows you to visualize motion data without needing MaestroDMX running. This is perfect for debugging and testing your motion sensor setup.

### Running Test Mode

```bash
# Test mode - shows real-time motion data visualization
python serial_to_osc_bridge.py --test

# Impact test mode - detailed impact detection debugging
python serial_to_osc_bridge.py --impact-test

# Or with specific serial port
python serial_to_osc_bridge.py /dev/ttyUSB0 --test
python serial_to_osc_bridge.py /dev/ttyUSB0 --impact-test

# Or with custom config file
python serial_to_osc_bridge.py --test --config /path/to/config.json
```

**Note:** The script automatically prioritizes USB serial ports over Bluetooth ports for better reliability.

### Test Mode Features

**📊 Real-time Data Display:**
- Raw accelerometer and gyroscope values
- Orientation angles (yaw, pitch, roll)
- Calculated motion metrics (magnitude, impact detection)
- OSC message preview (what would be sent to MaestroDMX)

**🎛️ Interactive Visualization:**
- Live updating display with timestamps
- Color-coded status indicators
- Impact detection alerts
- OSC message preview

**💡 Perfect for:**
- Debugging motion sensor calibration
- Testing wireless communication
- Understanding data ranges before MaestroDMX setup
- Demonstrating the system without lighting hardware

### Test Mode Output Example

```
🔬 SpaceWizard Motion Data Test Mode
======================================================================
Timestamp: 14:23:45

📊 RAW SENSOR DATA:
--------------------------------------------------
Accelerometer (g): |    0.123 |   -0.045 |    0.987
Gyroscope (°/s):   |   -10.2 |     4.1 |   120.5

🧭 ORIENTATION:
--------------------------------------------------
Yaw (degrees):     |   45.0°
Pitch (degrees):   |   30.0°
Roll (degrees):    |  -15.0°

⚡ CALCULATED METRICS:
--------------------------------------------------
Accel Magnitude (g): |    1.001
Gyro Magnitude (°/s): |   121.1

Impact Status:      | ✅ No impact

🎛️  OSC MESSAGES (Preview):
--------------------------------------------------
  → /global/brightness=0.501
  → /live/1/brightness=0.501
  → /live/1/speed=0.242
  → /live/1/excitement=0.242
  → /live/1/shape=0.125
  ... and 5 more messages

======================================================================
💡 Tip: Move your SpaceWizard device to see real-time motion data!
   Press Ctrl+C to exit test mode
```

### Impact Test Mode

The **impact test mode** provides detailed analysis of the impact detection logic to help debug false positives or sensitivity issues.

#### Running Impact Test Mode

```bash
# Impact test mode - detailed impact detection debugging
python serial_to_osc_bridge.py --impact-test

# Or with specific serial port
python serial_to_osc_bridge.py /dev/ttyUSB0 --impact-test
```

#### Impact Test Mode Features

**🔍 Detailed Analysis:**
- **Raw accelerometer values** in both raw and g-force units
- **Magnitude calculations** with baseline comparison
- **Threshold analysis** showing current vs. required values
- **Cooldown tracking** with time since last impact
- **Decision logic breakdown** with step-by-step evaluation

**🎯 Real-time Debugging:**
- **Live threshold monitoring** - see exactly when impacts trigger
- **Baseline deviation tracking** - understand sensor drift
- **Cooldown visualization** - see timing between impacts
- **Color-coded results** - green checkmarks for met conditions

**📊 Sample Impact Test Output:**

```
🎯 SpaceWizard Impact Detection Test Mode
================================================================================
Timestamp: 01:18:45

📊 ACCELEROMETER ANALYSIS:
------------------------------------------------------------
Raw Values:      | X: -0.104 | Y: -0.017 | Z:  0.490
In g-force:      | X: -0.006 | Y: -0.001 | Z:  0.030

Magnitude:       | Total: 0.502g
Baseline (1g):   | Expected: 1.000g
Deviation:       | Difference: 0.498g

🎯 IMPACT DETECTION ANALYSIS:
------------------------------------------------------------
Threshold:       | Raw:  8000 | In g:  0.488g
Cooldown:        |   2000ms remaining
Last Impact:     |      0ms ago

🔍 DECISION LOGIC:
------------------------------------------------------------
Deviation > Threshold: |   ❌ No (Deviation: 0.498g > 0.488g)
Cooldown Passed:      |   ✅ Yes (Time: 0ms > 2000ms)
Impact Triggered:     |   ❌ NO

✅ No impact detected - sensor readings are within normal range

================================================================================
💡 DEBUGGING TIPS:
   • Green checkmarks = condition met
   • Red X marks = condition not met
   • Adjust threshold in config.json if too sensitive/insensitive
   • Check if sensor is perfectly level (affects baseline)
   • Press Ctrl+C to exit impact test mode
```

**💡 Perfect for:**
- **Debugging false positives** - see exactly why impacts are triggering
- **Tuning sensitivity** - adjust threshold and cooldown values
- **Understanding sensor behavior** - learn how your hardware behaves
- **Validating calibration** - ensure sensor readings are stable

## Serial Protocol

The receiver Arduino outputs comprehensive MPU6050 data in this format:
```
AX=0.123 AY=0.034 AZ=0.987 GX=-10.2 GY=4.1 GZ=120.5 YAW=45.0 PITCH=30.0 ROLL=-15.0 IMP=0
```

Where:
- `AX, AY, AZ`: Accelerometer values (g-force, -16g to +16g)
- `GX, GY, GZ`: Gyroscope values (degrees/second, -2000 to +2000)
- `YAW`: Yaw angle (degrees, 0-360)
- `PITCH`: Pitch angle (degrees, -90 to +90)
- `ROLL`: Roll angle (degrees, -90 to +90)
- `IMP`: Impact detection flag (0/1) - triggers on significant acceleration changes

## Configuration & Tuning

### Using the Configuration File

The `config.json` file provides complete control over sensor-to-OSC mapping:

```json
{
  "accelerometer_x": {
    "enabled": true,
    "maestro_mapping": [
      {
        "osc_address": "/live/1/brightness",
        "scale_min": -2.0,
        "scale_max": 2.0,
        "invert": false,
        "description": "Left-right tilt controls brightness"
      }
    ]
  }
}
```

**Configuration Options:**
- **Enable/disable** specific sensor data sources
- **Map sensors** to any MaestroDMX OSC address
- **Set scaling ranges** for proper 0.0-1.0 value mapping
- **Configure thresholds** for trigger effects (strobe, blackout, etc.)
- **Customize sensitivity** and deadzone filtering

### Key Configuration Sections

#### MPU6050 Properties
- `accelerometer_x/y/z`: Linear acceleration (g-force)
- `gyroscope_x/y/z`: Angular velocity (degrees/second)
- `orientation_yaw/pitch/roll`: Device orientation angles
- `impact_detection`: Sudden movement detection

#### Maestro Settings
- `default_host`: MaestroDMX hostname/IP
- `default_port`: OSC port (default: 7672)
- `alternative_hosts`: Additional connection options

#### Data Processing
- `smoothing_factor`: Sensor data smoothing (0.0-1.0)
- `deadzone_threshold`: Minimum movement threshold
- `output_rate_hz`: Data transmission rate
- `enable_debug_output`: Verbose console output

### Quick Tuning Examples

**For More Sensitive Motion Response:**
```json
{
  "data_processing": {
    "deadzone_threshold": 0.02,
    "smoothing_factor": 0.05
  }
}
```

**For Impact-Triggered Effects:**
```json
{
  "impact_detection": {
    "threshold": 8000,
    "cooldown_ms": 2000
  }
}
```

**Impact Detection Tuning:**
- **Threshold**: Higher values (6000-10000) = less sensitive, fewer false positives
- **Cooldown**: Longer periods (1500-3000ms) = prevents rapid-fire impacts
- **Baseline filtering**: Accounts for sensor drift and gravity variations
- **Test with your hardware** - stationary board should not trigger impacts

**For Different OSC Mappings:**
```json
{
  "accelerometer_z": {
    "maestro_mapping": [
      {
        "osc_address": "/live/1/brightness",
        "scale_min": -1.0,
        "scale_max": 1.0
      }
    ]
  }
}
```

## Troubleshooting

### Serial Port Issues
- **Linux/macOS**: Check `/dev/ttyUSB*` or `/dev/ttyACM*` (preferred over Bluetooth)
- **Windows**: Check `COM3`, `COM4`, etc. in Device Manager
- **Auto-detection**: Script prioritizes USB serial ports over Bluetooth for reliability
- **Manual specification**: `python serial_to_osc_bridge.py /dev/ttyUSB0` to specify port
- Use `python -c "import serial.tools.list_ports; print(list(serial.tools.list_ports.comports()))"` to list ports

### MaestroDMX Connection Issues
1. Verify MaestroDMX is running
2. Check network connectivity (`ping maestro.local`)
3. Confirm OSC port settings in MaestroDMX
4. Try using IP address instead of hostname (`192.168.37.1` or `10.0.0.200`)

### Configuration File Issues
- **Auto-detection**: Script looks for `config.json` in current directory, script directory, and working directory
- **Manual path**: Use `--config /path/to/config.json` to specify custom location
- **Default fallback**: Uses built-in defaults if config file not found

### No Motion Response
1. Verify transmitter is sending data (check serial monitor on receiver)
2. Check Python script console output
3. Verify MaestroDMX OSC settings
4. Test with simple OSC sender (e.g., `python -c "from pythonosc.udp_client import SimpleUDPClient; c = SimpleUDPClient('maestro.local', 7672); c.send_message('/global/brightness', 1.0)"`)

## Hardware Setup & Troubleshooting


### nRF24L01 Radio Setup

**Critical Requirements:**
- **Stable 3.3V power supply** (separate from Arduino's 5V)
- **Proper antenna connection** (external antenna recommended for range)
- **Clean SPI communication** (no long wires, proper grounding)

**Common Issues:**
- **Range problems**: Check antenna orientation, avoid metal obstructions
- **Intermittent connection**: Verify power supply stability, check for electrical noise
- **No communication**: Ensure CE/CSN pins match between transmitter and receiver

### Arduino Compatibility

**Tested Boards:** Arduino Nano, Arduino Uno

**Pin Configuration:**
- I2C: A4 (SDA), A5 (SCL) - requires 4.7kΩ pull-up resistors
- SPI: D11 (MOSI), D12 (MISO), D13 (SCK)
- Radio: D9 (CE), D10 (CSN)
- Button: D6 (with internal pull-up)

### Power Supply Requirements

**Arduino:** Stable 5V supply (USB or external)
**nRF24L01:** Clean 3.3V supply (use separate regulator if needed)
**MPU6050:** 3.3V-5V compatible (use 3.3V for best performance)

**Power Issues:**
- **Brownouts**: Use adequate power supply, add capacitors
- **Radio instability**: Separate power rail for nRF24L01
- **Sensor drift**: Ensure stable voltage during operation

### Common Hardware Problems

#### I2C Communication Failures
- **Missing pull-up resistors** (add 4.7kΩ to SDA/SCL lines)
- **Wrong I2C address** (MPU6050 default is 0x68)
- **Long I2C wires** (keep under 30cm, use shielded cable)
- **Power supply noise** (add decoupling capacitors)

#### Radio Range Issues
- **Poor antenna connection** (ensure antenna is properly soldered)
- **Interference** (avoid 2.4GHz sources like WiFi, Bluetooth)
- **Power supply noise** (clean 3.3V essential for radio)
- **Physical obstructions** (metal objects block 2.4GHz signals)

#### MPU6050 Issues
- **No DMP initialization** (check I2C connection and pull-ups)
- **Inaccurate readings** (run calibration sketch)
- **Temperature sensitivity** (calibrate at operating temperature)
- **Vibration interference** (mount sensor securely)

## File Structure

```
Spacewizard Project:
├── mpu6050_calibration.ino             # MPU6050 calibration sketch (run first!)
├── mpu6050_calibration_simple.ino      # Simple calibration sketch (no external libraries)
└── maestro-osc-version/
    ├── Spacewizard2point0_osc_transmitter.ino   # Arduino transmitter (sends raw MPU6050 data)
    ├── Spacewizard2point0_osc_receiver.ino      # Arduino receiver (transmits real MPU6050 data)
    ├── serial_to_osc_bridge.py                  # Python OSC bridge (uses config.json)
    ├── config.json                              # Configuration file (MPU6050 to OSC mapping)
    ├── requirements.txt                         # Python dependencies
    └── README.md                               # This file
```

**Note**: The calibration sketches are in the root SpaceWizard folder and are required for both LED and OSC versions. Use `mpu6050_calibration_simple.ino` for the simplest setup.

## Related Projects

- **LED Version**: `../led-version/` - Original NeoPixel LED control version

## OSC Specification Reference

Based on [MaestroDMX OSC Specification v1.4.1](https://maestrodmx.freshdesk.com/support/solutions/articles/153000224681-open-sound-control-osc-specification-v1-4-1)

## License

Same as original SpaceWizard project.

## Mappable Variables (from MPU6050)

You can map any of these to OSC using `osc_mappings` in `config.json`:

- accelerometer_x: X-axis acceleration (g)
- accelerometer_y: Y-axis acceleration (g)
- accelerometer_z: Z-axis acceleration (g)
- gyroscope_x: X-axis angular velocity (°/s)
- gyroscope_y: Y-axis angular velocity (°/s)
- gyroscope_z: Z-axis angular velocity (°/s)
- orientation_yaw: Yaw angle (°)
- orientation_pitch: Pitch angle (°)
- orientation_roll: Roll angle (°)
- accel_magnitude: sqrt(AX^2 + AY^2 + AZ^2) (g)
- gyro_magnitude: sqrt(GX^2 + GY^2 + GZ^2) (°/s)
- deviation_from_baseline_g: |accel_magnitude - 1.0| (g)
- impact_triggered: boolean (True on impact event)

## Common MaestroDMX OSC Addresses

Examples from the MaestroDMX OSC spec (v1.4.1):
- /global/brightness (float 0.0–1.0)
- /live/N/brightness (float 0.0–1.0)
- /live/N/speed (float 0.0–1.0)
- /live/N/motion/speed (float 0.0–1.0)
- /live/N/excitement (float 0.0–1.0)
- /live/N/energy (float 0.0–1.0)
- /live/N/variance (float 0.0–1.0)
- /live/N/decay (float 0.0–1.0)
- /live/N/attack (float 0.0–1.0)
- /live/N/pattern/index (int)
- /live/N/palette/index (int)
- /triggers/strobe (boolean)

See full table: [MaestroDMX OSC v1.4.1](https://maestrodmx.freshdesk.com/support/solutions/articles/153000224681-open-sound-control-osc-specification-v1-4-1)

## Flat OSC Mappings (config.json)

Edit `osc_mappings` to quickly change routing. Each entry supports:
- enabled (bool): turn mapping on/off
- source (string): variable name above; prefix with `abs:` to map absolute value
- osc_address (string): Maestro OSC path
- scale_min/scale_max (numbers): input range to map into 0.0–1.0
- invert (bool): optional, flips 0.0–1.0
- threshold (bool): treat source as boolean trigger

Example:
```json
{
  "osc_mappings": [
    { "enabled": true,  "source": "gyroscope_z",     "osc_address": "/live/1/speed",      "scale_min": -500,  "scale_max": 500,  "invert": false },
    { "enabled": false, "source": "abs:gyroscope_z", "osc_address": "/live/1/speed",      "scale_min": 0,     "scale_max": 500,  "invert": false },
    { "enabled": true,  "source": "accel_magnitude", "osc_address": "/global/brightness", "scale_min": 0.8,   "scale_max": 2.0,  "invert": false },
    { "enabled": true,  "source": "impact_triggered","osc_address": "/triggers/strobe",  "threshold": true }
  ]
}
```
