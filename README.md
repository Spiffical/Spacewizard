# SpaceWizard Project

A wireless motion-controlled lighting system with two different implementations for controlling lights using MPU6050 sensor data and nRF24L01 wireless communication.

## Project Overview

SpaceWizard uses motion sensors to control lighting effects wirelessly. The system consists of:
- **Transmitter unit** with MPU6050 motion sensor and button controls
- **Receiver unit** that processes the wireless data
- **Two different lighting control systems** (selectable based on your hardware)

## Project Versions

This repository contains two complete implementations of the SpaceWizard system:

### 🎨 [LED Version](./led-version/) - Direct NeoPixel Control
- **Controls NeoPixel LED strips directly**
- Motion-responsive patterns that react to device orientation
- Multiple animation modes with automated and manual control
- Perfect for DIY LED installations and Arduino-based projects

**Features:**
- Real-time LED position control based on device roll
- Color effects based on pitch and yaw angles
- Impact-reactive brightness for dynamic effects
- Button-based mode switching (5 press modes + hold mode)

### 🎭 [Maestro OSC Version](./maestro-osc-version/) - Professional Lighting Control
- **Controls MaestroDMX software via OSC (Open Sound Control)**
- Designed for professional lighting systems and installations
- Converts motion data to OSC messages over network
- Compatible with DMX lighting fixtures and professional show control

**Features:**
- Network-based control using OSC protocol
- Maps motion data to MaestroDMX parameters
- Professional lighting effects (strobe, brightness, speed, excitement)
- Python bridge for easy integration and customization

## Hardware Requirements

### Common Components (Both Versions)
- **Transmitter Arduino**: Nano/UNO with MPU6050 sensor and nRF24L01 radio
- **Motion Sensor**: MPU6050 6-axis accelerometer/gyroscope
- **Wireless Module**: nRF24L01 radio transceiver (2.4GHz)
- **Power**: 5V for Arduino, 3.3V for radio modules

### Version-Specific Components

#### LED Version Additional Hardware
- **Receiver Arduino**: Nano/UNO with nRF24L01 radio
- **LED Strip**: NeoPixel/WS2812B LED strip (40 LEDs recommended)
- **Power Supply**: Sufficient for LED strip current requirements

#### Maestro OSC Version Additional Hardware
- **Receiver Arduino**: Nano/UNO with nRF24L01 radio (USB connected to computer)
- **Computer/Laptop**: Running Python 3.6+ with MaestroDMX software
- **Network**: Connection between computer and MaestroDMX system

## MPU6050 Calibration (Required for Both Versions)

**Every MPU6050 sensor requires individual calibration** for accurate motion tracking. This is a one-time setup process that determines the unique offset values for your specific sensor.

### Calibration Process

1. **Upload calibration sketch**: Use `mpu6050_calibration.ino` (located in the root folder)
2. **Place sensor flat and motionless** on a stable, non-magnetic surface
3. **Run calibration**: Upload the sketch and open Serial Monitor (115200 baud)
4. **Wait ~30 seconds** for calibration to complete - sensor must remain completely still
5. **Copy offset values** to your transmitter code in both versions:
   ```cpp
   mpu.setXAccelOffset(-1449);  // Replace with your calibrated values
   mpu.setYAccelOffset(2673);   // Replace with your calibrated values
   mpu.setZAccelOffset(692);    // Replace with your calibrated values
   mpu.setXGyroOffset(93);      // Replace with your calibrated values
   mpu.setYGyroOffset(21);      // Replace with your calibrated values
   mpu.setZGyroOffset(-31);     // Replace with your calibrated values
   ```

**Important Notes:**
- Each MPU6050 sensor has unique offset values
- Run calibration with your specific sensor before first use
- Place sensor on vibration-free, non-magnetic surface
- Ensure stable power supply during calibration
- Values are different for each sensor - never reuse values from another MPU6050

## Quick Start

### For LED Version
1. **Calibrate MPU6050** (see Calibration section above)
2. Navigate to `led-version/` folder
3. Upload transmitter and receiver Arduino code
4. Connect NeoPixel LED strip to receiver
5. Power on both units and enjoy motion-controlled lighting!

### For Maestro OSC Version
1. **Calibrate MPU6050** (see Calibration section above)
2. Navigate to `maestro-osc-version/` folder
3. Install Python dependencies: `pip install -r requirements.txt`
4. Upload Arduino receiver code (transmitter code is the same)
5. Run Python script: `python serial_to_osc_bridge.py`
6. Control MaestroDMX lights with motion!

## Technical Architecture

```
Motion Data Flow:
[MPU6050 Sensor] → [Arduino Transmitter] → [nRF24L01 Radio] → [Arduino Receiver] → [Processing] → [Light Control]
```

### LED Version Data Flow
```
[Motion Sensors] → [Wireless] → [LED Patterns] → [NeoPixel Strip]
```

### Maestro OSC Version Data Flow
```
[Motion Sensors] → [Wireless] → [Serial Output] → [Python OSC Bridge] → [Network OSC] → [MaestroDMX] → [DMX Lights]
```

## Repository Structure

```
Spacewizard/
├── README.md                           # This main project overview
├── mpu6050_calibration.ino             # MPU6050 calibration sketch (advanced version with libraries)
├── mpu6050_calibration_simple.ino      # MPU6050 calibration sketch (simple version, no external libraries)
├── led-version/                        # Direct NeoPixel LED control version
│   ├── Spacewizard2point0_txMULTI.ino         # Transmitter Arduino code
│   ├── Spacewizard2point0_rx.ino             # Receiver with NeoPixel control
│   └── README.md                               # LED version documentation
└── maestro-osc-version/                # MaestroDMX OSC control version
    ├── Spacewizard2point0_osc_receiver.ino    # Serial-output receiver
    ├── serial_to_osc_bridge.py                # Python OSC bridge
    ├── config.json                            # MPU6050 to OSC mapping configuration
    ├── requirements.txt                       # Python dependencies
    └── README.md                             # OSC version documentation
```

## Choosing the Right Version

### Use LED Version If You Want:
- Direct Arduino control of LED strips
- Simple DIY lighting projects
- No computer required for operation
- Immediate visual feedback

### Use Maestro OSC Version If You Want:
- Professional DMX lighting control
- Integration with existing lighting systems
- Network-based control and show programming
- Advanced lighting effects and programming
- Scalable multi-fixture installations

## Development History

- **Original LED Version**: Started as a wireless motion-controlled LED project
- **Maestro OSC Version**: Extended to control professional lighting systems via OSC
- **Both versions** maintain the same wireless protocol for compatibility

## Support & Documentation

Each version has its own comprehensive README with:
- Detailed setup instructions
- Hardware connection diagrams
- Configuration options
- Troubleshooting guides
- Technical specifications

## License

This project is open source. See individual version folders for specific license information.

---

**Ready to start?** Choose your version above and follow the setup instructions!
