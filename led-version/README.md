# SpaceWizard LED Version (Original)

This is the original SpaceWizard project that directly controls **NeoPixel LED strips** using motion data from an MPU6050 sensor via wireless nRF24L01 communication.

## Overview

A wireless motion-controlled LED lighting system featuring:
- **MPU6050 accelerometer/gyroscope** for motion detection
- **nRF24L01 wireless communication** between transmitter and receiver
- **NeoPixel LED strip control** with multiple animation patterns
- **Button-based mode switching** for different effects

## Hardware Requirements

### Transmitter Unit
- Arduino Nano/UNO
- MPU6050 6-axis accelerometer/gyroscope (I2C)
- nRF24L01 radio transceiver module (SPI)
- Push button for mode switching

### Receiver Unit
- Arduino Nano/UNO
- nRF24L01 radio transceiver module (SPI)
- NeoPixel LED strip (40 LEDs recommended)

## Features

### Motion-Responsive Patterns (Hold Mode 0)
- **Scanner Pattern**: LED position follows device roll angle
- **Full Color Pattern**: Color hue based on device pitch with impact-reactive brightness
- **Variable Color Pattern**: Progressive color fill based on roll angle and hue from pitch

### Automated Patterns (Hold Mode 1)
- **Rainbow Cycle**: Smooth color cycling animation
- **Theater Chase**: Alternating color chase effect
- **Color Wipe**: Progressive color filling
- **Scanner**: Back-and-forth moving light with fade trails
- **Fade**: Smooth color transitions

### Control Modes
- **Press Modes (0-4)**: Different pattern variations
- **Hold Mode**: Toggle between manual (motion-responsive) and automated patterns

## Installation & Setup

### 1. Upload Arduino Code

1. **Transmitter**: Upload `Spacewizard2point0_txMULTI.ino` to transmitter Arduino
2. **Receiver**: Upload `Spacewizard2point0_rx.ino` to receiver Arduino

### 2. Hardware Connections

#### Transmitter
- MPU6050 → I2C (A4/A5 on Arduino Nano)
- nRF24L01 → SPI (D9=CE, D10=CS, D11=MOSI, D12=MISO, D13=SCK)
- Button → Digital Pin 6 with pull-up resistor

#### Receiver
- nRF24L01 → SPI (D9=CE, D10=CS, D11=MOSI, D12=MISO, D13=SCK)
- NeoPixel Data → Digital Pin 8

### 3. MPU6050 Calibration (Required)

**Every MPU6050 sensor requires individual calibration** for accurate motion tracking. See the [main project README](../README.md#mpu6050-calibration-required-for-both-versions) for detailed calibration instructions. Two calibration sketches are available in the root SpaceWizard folder:
- `mpu6050_calibration.ino` (advanced version)
- `mpu6050_calibration_simple.ino` (simple version, recommended)

## Usage

1. **Power on transmitter** - MPU6050 will initialize and begin sending motion data
2. **Power on receiver** - NeoPixel strip will initialize and wait for wireless data
3. **Motion control**:
   - **Roll**: Controls LED position in scanner modes
   - **Pitch**: Controls color hue and brightness effects
   - **Yaw**: General motion intensity affects brightness
4. **Button controls**:
   - **Short press**: Cycle through press modes (0-4)
   - **Long press**: Toggle between manual and automated pattern modes

## LED Animation Patterns

### Manual Control Mode (Hold = 0)
- **Press Mode 0**: Red scanner follows roll angle
- **Press Mode 1**: Full rainbow with impact-reactive brightness
- **Press Mode 2**: Variable color sections based on roll/pitch

### Automated Mode (Hold = 1)
- **Press Mode 0**: Yellow/blue theater chase
- **Press Mode 1**: Rainbow cycle animation
- **Press Mode 2**: Red scanner with fade trails

## Technical Details

### Wireless Communication
- **Protocol**: nRF24L01 proprietary protocol
- **Frequency**: 2.4 GHz ISM band
- **Data Rate**: 250 Kbps for better range
- **Addressing**: Multiple pipes for redundancy (0xE8E8F0F0E1, 0xEEFDFAF50DF, 0xEEFDFAF50E2)

### Motion Processing
- **Sensor**: MPU6050 with DMP (Digital Motion Processor)
- **Sample Rate**: ~40 Hz (adjustable)
- **Data**: Yaw, pitch, roll angles + acceleration data
- **Filtering**: Running average for smoother transitions

### LED Control
- **Library**: Adafruit NeoPixel
- **LED Count**: 40 LEDs (configurable)
- **Color Depth**: 24-bit RGB
- **Refresh Rate**: Hardware-dependent (WS2812B ~400 Hz)

## Troubleshooting

### Wireless Connection Issues
1. Verify both nRF24L01 modules use the same address pipes
2. Check antenna orientation and distance (max ~100m line-of-sight)
3. Ensure 3.3V power supply for nRF24L01 modules

### MPU6050 Issues
1. Check I2C connections (A4/A5) and pull-up resistors
2. Verify calibration offsets for your specific MPU6050 unit
3. Monitor DMP initialization in serial output

### LED Issues
1. Verify NeoPixel data pin and power connections
2. Check for sufficient power supply (WS2812B LEDs are power-hungry)
3. Ensure proper ground connections

## File Structure

```
SpaceWizard Project:
├── mpu6050_calibration.ino         # MPU6050 calibration sketch (required for both versions)
├── led-version/                    # Direct NeoPixel LED control version
│   ├── Spacewizard2point0_txMULTI.ino    # Transmitter Arduino code
│   ├── Spacewizard2point0_rx.ino         # Receiver with NeoPixel control
│   └── README.md                        # This file
└── maestro-osc-version/            # MaestroDMX OSC control version
    └── [other files...]
```

**Note**: The calibration sketch is in the root SpaceWizard folder and is required for both versions.

## Related Projects

- **Maestro OSC Version**: `../maestro-osc-version/` - Controls MaestroDMX via OSC instead of direct LED control

## Version History

- **v2.0**: Enhanced MPU6050 DMP integration, improved button handling, multiple animation patterns
- **v1.0**: Basic wireless motion control with simple LED patterns

## License

Original SpaceWizard project license applies.
