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
```

## Serial Protocol

The receiver Arduino outputs comprehensive MPU6050 data in this format:
```
AX=0.123 AY=0.034 AZ=0.987 GX=-10.2 GY=4.1 GZ=120.5 YAW=45.0 PITCH=30.0 ROLL=-15.0 IMP=0 BTN=0
```

Where:
- `AX, AY, AZ`: Accelerometer values (g-force, -16g to +16g)
- `GX, GY, GZ`: Gyroscope values (degrees/second, -2000 to +2000)
- `YAW`: Yaw angle (degrees, 0-360)
- `PITCH`: Pitch angle (degrees, -90 to +90)
- `ROLL`: Roll angle (degrees, -90 to +90)
- `IMP`: Impact detection flag (0/1)
- `BTN`: Button state (0-15, combination of press and hold modes)

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
    "threshold": 600,
    "cooldown_ms": 300
  }
}
```

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
- **Linux/macOS**: Check `/dev/ttyUSB*` or `/dev/ttyACM*`
- **Windows**: Check `COM3`, `COM4`, etc. in Device Manager
- Use `python -c "import serial.tools.list_ports; print(list(serial.tools.list_ports.comports()))"` to list ports

### MaestroDMX Connection Issues
1. Verify MaestroDMX is running
2. Check network connectivity (`ping maestro.local`)
3. Confirm OSC port settings in MaestroDMX
4. Try using IP address instead of hostname

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
└── maestro-osc-version/
    ├── Spacewizard2point0_osc_receiver.ino    # Arduino receiver (transmits real MPU6050 data)
    ├── serial_to_osc_bridge.py                # Python OSC bridge (uses config.json)
    ├── config.json                            # Configuration file (MPU6050 to OSC mapping)
    ├── requirements.txt                       # Python dependencies
    └── README.md                             # This file
```

**Note**: The calibration sketch is in the root SpaceWizard folder and is required for both LED and OSC versions.

## Related Projects

- **LED Version**: `../led-version/` - Original NeoPixel LED control version

## OSC Specification Reference

Based on [MaestroDMX OSC Specification v1.4.1](https://maestrodmx.freshdesk.com/support/solutions/articles/153000224681-open-sound-control-osc-specification-v1-4-1)

## License

Same as original SpaceWizard project.
