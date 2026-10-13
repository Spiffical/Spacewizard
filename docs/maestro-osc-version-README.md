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

The system uses:
- `config.json` for runtime settings (host, impact tuning, data processing)
- `osc_mappings_*.json` for source-to-OSC mapping constants/routes

This split makes OSC routing easy to change without touching operational settings.

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

### Configuration Files

- `config.json` handles runtime behavior (network, calibration, filtering, impact detection)
- `osc_mappings_pickup.json` is the default mapping profile for pickup interaction mode
- `osc_mappings_pickup_cinematic.json` is an alternate profile
- `osc_mappings.json` remains as a legacy profile

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
python run_osc_bridge.py

# Or specify custom settings
python run_osc_bridge.py /dev/ttyUSB0 maestro.local 7672

# Pick a mapping profile from config.json (pickup / pickup-cinematic / legacy)
python run_osc_bridge.py --mapping-profile pickup-cinematic

# Or point directly at a mapping file
python run_osc_bridge.py --mapping-file osc_mappings_pickup.json

# Test mode - visualize motion data without MaestroDMX connection
python run_osc_bridge.py --test
```

You can also run it as a module: `python -m bridge`.

## Experimental Synth

There is now a first SuperCollider synth sketch for the pickup piece in `synth/`.

- `synth/SpaceWizardSynth.scd` contains the main patch and OSC hooks for later integration
- `synth/SpaceWizardNoOscTests.scd` contains quick audition tests that do not require OSC or sensor hardware
- `synth/README.md` explains the listening tests and intended parameter mapping
- `start_spacewizard_live.sh` restarts the local synth + bridge stack and auto-detects the receiver port

This lets you audition the sound world on the MacBook before wiring the sensor stream into it.

The ambient demo GUI now includes four switchable variants: `Flow`, `Harmonic Chords`, `Drone Texture`, and `Forest Canopy`.

In the live rig, `ambient-flow` can now start from any one of those ambient variants, and yaw smoothly blends around the full four-mode ring. That synth yaw now follows the DMP heading only once the candle is horizontal enough, so upright twisting no longer moves the ambient ring. Long-axis twisting is now reserved for synth roll instead. Halfway between neighboring ambient variants, both run at equal level. The `drone` portion of that ring also layers in low pentatonic bass notes from the short-note synth mode.

To boot the whole local live rig with one command, run:

```bash
./start_spacewizard_live.sh
```

That script will:
- stop any existing local SpaceWizard bridge process
- restart local SuperCollider language/audio processes
- auto-detect the receiver serial port
- start the synth patch headlessly
- start the bridge in low-latency `--test --synth` mode

Useful options:

```bash
./start_spacewizard_live.sh --verbose-test
./start_spacewizard_live.sh --mode ambient-flow
./start_spacewizard_live.sh --mode ambient-flow --ambient-variant forest
./start_spacewizard_live.sh --mode smooth-sine
./start_spacewizard_live.sh --mode smooth-sine --smooth-preset quiet-pentatonic
./start_spacewizard_live.sh --mode smooth-sine --smooth-preset quiet-pentatonic --smooth-mix-gui
./start_spacewizard_live.sh --mode smooth-sine --status-gui
./start_spacewizard_live.sh --mode reese-bass
./start_spacewizard_live.sh --mode wow-voice
./start_spacewizard_live.sh --port /dev/cu.usbserial-210
./start_spacewizard_live.sh --stop
```

`--status-gui` opens a live SuperCollider status window that shows current synth-control values and parameters, including calibrated pitch, long-axis roll, and the horizontal-only yaw state.

Live sound settings are configured in `sound_presets.scd`. That file is loaded at boot and is organized by triggered sound preset:

- `smoothSine.standard`
- `smoothSine.quietPentatonic`
- `ambientFlow.flow`, `ambientFlow.chordal`, `ambientFlow.drone`, `ambientFlow.forest`
- `auduino.standard`
- `reeseBass.standard`
- `wowVoice.standard`

Use it for tuned values such as quiet synth volume, ambient volume, choir volume, choir/synth blend, fade times, and roll-to-crunch/reverb ranges. It is a SuperCollider config file so each option has inline comments and ranges.

Runtime logs go to `maestro-osc-version/.runtime/logs/`.

To drive the SuperCollider patch from the live receiver bridge, run:

```bash
python run_osc_bridge.py /dev/cu.usbserial-210 --test --synth
```

This keeps Maestro disabled while still sending OSC to the local SuperCollider patch on `127.0.0.1:57120`.
When `--test` and `--synth` are used together, the bridge now suppresses the heavy full-screen terminal redraw by default to reduce control latency. Add `--verbose-test` if you want the old visualization back while debugging.

The synth bridge now defaults to a calibrated accelerometer-derived tilt mapping for the candle's current bottom-mounted sensor orientation, rather than using the raw transmitted `PITCH`/`ROLL` values directly.

To inspect the derived synth values during a single pickup gesture, run:

```bash
python capture_pickup_synth_trace.py --port /dev/cu.usbserial-210
```

That guided trace walks through flat on table, lift to `+90`, hold, return to `0`, and place-flat again, while logging `synth_pitch_deg`, `synth_roll_deg`, norms, gate, and activity to a CSV for later analysis.
The final place-flat stage is intentionally longer than the others so it can capture the current `3s` interaction rest hold after placement motion settles.

To diagnose the current yaw-vs-roll behavior more directly, run:

```bash
python capture_axis_mapping_trace.py --port /dev/cu.usbserial-210
```

That guided trace starts with the candle horizontal, then walks through long-axis roll while horizontal, raising pitch toward `+90`, long-axis roll again at high pitch, returning to horizontal, and finally changing yaw by turning your body while keeping the candle horizontal. It logs both raw transmitter angles and derived synth values such as `synth_yaw_norm`, `synth_roll_deg`, `synth_yaw_active`, `synth_horizontal_strength`, the projected long-axis gyro value, and raw quaternion components to CSV.

The current quaternion roll path is referenced to the session's starting orientation, so begin a capture or live run from the pose you want to count as approximately `0` roll before moving.

For the narrower pitch-versus-roll coupling check, run:

```bash
python capture_axis_mapping_trace.py --protocol pitch-roll-coupling --port /dev/cu.usbserial-210
```

That protocol starts horizontal, raises pitch to `+90`, returns to `0`, rolls to about `+90` while horizontal, then repeats the pitch-up and pitch-down motion while holding that rolled orientation. Use it when roll appears to move during pure pitch changes, or when a held roll offset seems unstable.

The full yaw/roll fix now depends on quaternion data from the Arduino pair. After pulling the latest code, reflash both [Spacewizard2point0_osc_transmitter.ino](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/Spacewizard2point0_osc_transmitter/Spacewizard2point0_osc_transmitter.ino) and [Spacewizard2point0_osc_receiver.ino](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/Spacewizard2point0_osc_receiver/Spacewizard2point0_osc_receiver.ino) before rerunning the axis diagnostic.

## Test Mode

The OSC bridge includes a **test mode** that allows you to visualize motion data without needing MaestroDMX running. This is perfect for debugging and testing your motion sensor setup.

### Running Test Mode

```bash
# Test mode - shows real-time motion data visualization
python run_osc_bridge.py --test

# Impact test mode - detailed impact detection debugging
python run_osc_bridge.py --impact-test

# Or with specific serial port
python run_osc_bridge.py /dev/ttyUSB0 --test
python run_osc_bridge.py /dev/ttyUSB0 --impact-test

# Or with custom config file
python run_osc_bridge.py --test --config /path/to/config.json
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
python run_osc_bridge.py --impact-test

# Or with specific serial port
python run_osc_bridge.py /dev/ttyUSB0 --impact-test
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
AX=0.123 AY=0.034 AZ=0.987 GX=-10.2 GY=4.1 GZ=120.5 YAW=45.0 PITCH=30.0 ROLL=-15.0 QW=0.9980 QX=0.0120 QY=-0.0440 QZ=0.0320 IMP=0
```

Where:
- `AX, AY, AZ`: Accelerometer values (g-force, -16g to +16g)
- `GX, GY, GZ`: Gyroscope values (degrees/second, -2000 to +2000)
- `YAW`: Yaw angle (degrees, 0-360)
- `PITCH`: Pitch angle (degrees, -90 to +90)
- `ROLL`: Roll angle (degrees, -90 to +90)
- `QW, QX, QY, QZ`: Normalized quaternion components from the MPU6050 DMP
- `IMP`: Impact detection flag (0/1) - triggers on significant acceleration changes

## Configuration & Tuning

### Using the Configuration Files

`config.json` points to a separate mapping file:

```json
{
  "mapping_file": "osc_mappings_pickup.json",
  "mapping_profiles": {
    "pickup": "osc_mappings_pickup.json",
    "pickup-cinematic": "osc_mappings_pickup_cinematic.json",
    "legacy": "osc_mappings.json"
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

#### Mapping Files
- `mapping_file`: Default mappings file if no override is provided
- `mapping_profiles`: Named mapping sets you can select with `--mapping-profile`
- `mapping_constants.sources`: Reusable source + scaling definitions
- `mapping_constants.osc_addresses`: Reusable OSC address aliases
- `osc_routes`: Active route wiring (source constant -> OSC constant)

#### Maestro Settings
- `default_host`: MaestroDMX hostname/IP
- `default_port`: OSC port (default: 7672)
- `alternative_hosts`: Additional connection options

#### Data Processing
- `smoothing_factor`: Sensor data smoothing (0.0-1.0)
- `deadzone_threshold`: Minimum movement threshold
- `output_rate_hz`: Data transmission rate
- `enable_debug_output`: Verbose console output

#### Interaction Mode
- `interaction_mode.enabled`: Turn pickup/motion gate on/off
- `accel_on_g` / `gyro_on_dps`: Motion thresholds to turn lights on
- `accel_off_g` / `gyro_off_dps`: Lower thresholds to turn lights off
- `start_hold_ms`: Motion must exceed on-thresholds for this long before turning on
- `rest_hold_ms`: How long motion must stay below off-thresholds before lights turn off
- `stable_rest_enabled` / `stable_accel_delta_g` / `stable_hold_ms`: Also allow rest when accel magnitude stays nearly unchanged for a sustained period
- `baseline_alpha` / `baseline_update_gyro_max_dps`: Adaptive accel baseline tracking while resting
- `active_brightness` / `rest_brightness`: Global brightness values for active/rest states

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

**For Pickup Interaction Mode (lights on only while moving):**
```json
{
  "interaction_mode": {
    "enabled": true,
    "accel_on_g": 0.035,
    "accel_off_g": 0.02,
    "gyro_on_dps": 6.0,
    "gyro_off_dps": 2.2,
    "start_hold_ms": 90,
    "rest_hold_ms": 3000,
    "stable_rest_enabled": true,
    "stable_accel_delta_g": 0.006,
    "stable_hold_ms": 1800,
    "stable_gyro_max_dps": 2.2,
    "baseline_alpha": 0.03,
    "baseline_update_gyro_max_dps": 2.5,
    "active_brightness": 1.0,
    "rest_brightness": 0.0
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

### Input loss and recovery

The bridge sends explicit off commands at startup, during missing/invalid input,
and on normal shutdown (including SIGTERM/SIGHUP). Its default valid-frame timeout
is `data_processing.input_timeout_seconds: 0.5`. Safe-state commands repeat every
0.2 seconds until fresh data returns. The synth additionally closes its motion gate
and cancels stand prompts after 0.75 seconds without live control; the normal sound
fade-out still applies. Neither mechanism can deliver an off command to Maestro
while the network to Maestro itself is unavailable.

Serial removal/startup absence is retried every second. An explicit port is retried
at the same path (prefer `/dev/serial/by-id/...` on the Pi); automatic discovery is
repeated if no port was specified. OSC destinations recover independently.

The new firmware adds `SEQ` and `VALID` to serial frames without changing `FMT=2`
sensor scaling. The receiver accepts both the original 28-byte radio payload and
the new 32-byte payload. Repeated sequence numbers and invalid samples do not refresh
the bridge timeout. **Both updated Arduino sketches must be flashed to gain sensor
freshness detection and the complete firmware recovery path.** Legacy transmitters
remain usable, but a frozen sensor that keeps emitting legacy packets cannot be
distinguished from a live sensor by the bridge.

The production runner checks synth and bridge heartbeat files as well as PIDs.
If either heartbeat stops for over ten seconds, the service restarts the rig.
The synth heartbeat stops when its audio server stops. A candle that is switched
off keeps outputs off and does not itself cause restart loops.

Local regression checks (from the repository root):

```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python tests/run_synth_reliability.py
.venv/bin/python tests/run_synth_integration.py --output-device 'BlackHole 2ch'
arduino-cli compile --fqbn arduino:avr:nano maestro-osc-version/Spacewizard2point0_osc_receiver
arduino-cli compile --fqbn arduino:avr:nano maestro-osc-version/Spacewizard2point0_osc_transmitter
```

The synth integration check uses private OSC ports and a specified virtual audio
output. On Linux, set `SCLANG_BIN`/`QT_QPA_PLATFORM=offscreen` as needed and choose an
appropriate test output. Tests do not replace physical RF, power-fault, Maestro,
and speaker checks. Sensor scaling and sensitivity need a separate candle-on
calibration session; the existing tuned units are intentionally retained here.

### Serial Port Issues
- **Linux/macOS**: Check `/dev/ttyUSB*` or `/dev/ttyACM*` (preferred over Bluetooth)
- **Windows**: Check `COM3`, `COM4`, etc. in Device Manager
- **Auto-detection**: Script prioritizes USB serial ports over Bluetooth for reliability
- **Manual specification**: `python run_osc_bridge.py /dev/ttyUSB0` to specify port
- Use `python -c "import serial.tools.list_ports; print(list(serial.tools.list_ports.comports()))"` to list ports

### MaestroDMX Connection Issues
1. Verify MaestroDMX is running
2. Check network connectivity (`ping maestro.local`)
3. Confirm OSC port settings in MaestroDMX
4. Try using IP address instead of hostname (`192.168.37.1` or `10.0.0.200`)

### Configuration File Issues
- **Auto-detection**: Script looks for `config.json` and selected `osc_mappings_*.json` in current directory, project directory, and working directory
- **Manual path**: Use `--config /path/to/config.json` to specify custom location
- **Profile selection**: Use `--mapping-profile pickup` (or `pickup-cinematic`, `legacy`)
- **File override**: Use `--mapping-file /path/to/osc_mappings_custom.json`
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
    ├── run_osc_bridge.py                        # Preferred CLI entrypoint
    ├── serial_to_osc_bridge.py                  # Legacy CLI wrapper (still works)
    ├── bridge/                                  # Organized Python modules
    │   ├── app.py                               # Main runtime loop
    │   ├── config.py                            # Config + mapping loaders
    │   ├── metrics.py                           # Motion/impact calculations
    │   ├── osc.py                               # Mapping resolution + OSC send
    │   ├── serial_input.py                      # Serial parsing + port detect
    │   └── display.py                           # Test/impact terminal views
    ├── config.json                              # Runtime settings
    ├── osc_mappings_pickup.json                 # Default pickup interaction mappings
    ├── osc_mappings_pickup_cinematic.json       # Alternate pickup mappings
    ├── osc_mappings.json                        # Legacy mappings
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

You can map any of these to OSC using `mapping_constants.sources` in your selected `osc_mappings_*.json` profile:

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
- pickup_triggered: boolean (True for one frame when object is picked up)
- interaction_active: boolean (True while object is in active motion)
- interaction_gate: float (0.0-1.0, intended for global brightness gate)
- interaction_activity: float (0.0-1.0 activity amount while active)

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

## Route-based OSC Mappings (osc_mappings_*.json)

For fast remapping, use `mapping_constants` + `osc_routes`:
- Put reusable sensor transforms in `mapping_constants.sources`
- Put reusable OSC paths in `mapping_constants.osc_addresses`
- In `osc_routes`, connect `source` constant names to `osc` constant names

This makes swaps easy (for example, change one route from `gyro_z` to `gyro_y` without rewriting ranges/addresses).

Example:
```json
{
  "mapping_constants": {
    "sources": {
      "gyro_z": { "source": "gyroscope_z", "scale_min": -500, "scale_max": 500 },
      "gyro_y": { "source": "gyroscope_y", "scale_min": -500, "scale_max": 500 },
      "impact_event": { "source": "impact_triggered", "threshold": true }
    },
    "osc_addresses": {
      "live_speed": "/live/1/speed",
      "trigger_strobe": "/triggers/strobe"
    }
  },
  "osc_routes": [
    { "enabled": true, "source": "gyro_z", "osc": "live_speed" },
    { "enabled": true, "source": "impact_event", "osc": "trigger_strobe" }
  ]
}
```

Notes:
- `osc_routes` supports overrides per route (`scale_min`, `scale_max`, `invert`, `threshold`)
- `source` still supports `abs:` prefixes (for example `abs:gyroscope_z`)
- Legacy `osc_mappings` is still supported
