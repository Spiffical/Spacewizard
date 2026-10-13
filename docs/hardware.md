# Hardware

## Parts

- **Candle (transmitter):** Arduino Nano, MPU6050 (GY-521 board), nRF24L01, battery.
- **Receiver:** Arduino Nano and nRF24L01, connected to the Pi by USB.
- **Raspberry Pi 4** running Raspberry Pi OS, with audio out to speakers.
- **MaestroDMX** with DMX fixtures.

## Wiring

nRF24L01, on both Nanos:

| nRF24L01 | Nano |
|---|---|
| VCC | 3.3 V, with a 10 to 100 µF capacitor across VCC and GND |
| GND | GND |
| CE | D9 |
| CSN | D10 |
| SCK | D13 |
| MOSI | D11 |
| MISO | D12 |

MPU6050, on the candle's Nano only: VCC to 5 V, GND to GND, SDA to A4, SCL to A5, INT to D2.

A loose or bent USB jack on the receiver shows up as random dropouts and as the Pi
failing to recognise the board, so use a sturdy cable and strain relief.

## Libraries and flashing

Arduino AVR core, RF24 by TMRh20 (1.5.0), and MPU6050 by Electronic Cats (1.4.4, which
includes I2Cdev). With `arduino-cli`:

```bash
arduino-cli compile --fqbn arduino:avr:nano firmware/transmitter
arduino-cli upload -p /dev/cu.usbserial-XXXX --fqbn arduino:avr:nano:cpu=atmega328old firmware/transmitter
```

Many CH340 Nano clones need the `atmega328old` bootloader setting. Flash
`firmware/receiver` the same way.

## Calibrating the MPU6050

1. Flash `firmware/calibration/mpu6050_calibration` with the sensor flat and still.
   `mpu6050_calibration_simple` needs no libraries.
2. Open the serial monitor at 115200 baud and wait about 30 seconds.
3. Copy the printed offsets into the `MPU_*_OFFSET` defines in
   `firmware/transmitter/transmitter.ino` and reflash.

Known issue: the transmitter writes these offsets before it starts the DMP, and the
MPU6050 library's DMP start-up resets the chip, so the offsets currently have no effect.
The gyro shows a few degrees per second of bias at rest. Writing the offsets after
`mpu.dmpInitialize()` fixes it.

## Radio and serial format

The receiver prints one line per packet at 230400 baud, for example:

```
FMT=2 AX=454 AY=-469 AZ=8134 GX=-59 GY=9 GZ=-2 YAW=4 PITCH=3 ROLL=-3 QW=16358 QX=-488 QY=-439 QZ=-642 IMP=0 SEQ=5825 VALID=1
```

Each sample carries a sequence number and a validity flag, so the bridge can tell a
frozen sensor or a lost radio link from a candle at rest.

## MaestroDMX

MaestroDMX listens for OSC on UDP port 7672, at 10.0.0.200 over Ethernet or 192.168.37.1
on its own Wi-Fi hotspot. The bridge drives `/global/brightness` and the `/live/1/...`
controls listed in `config/osc_mappings_pickup.json`.
