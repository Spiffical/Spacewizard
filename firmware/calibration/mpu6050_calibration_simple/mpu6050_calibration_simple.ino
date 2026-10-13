/*
 * Simple MPU6050 Calibration Sketch - No External Libraries Required
 *
 * This sketch calibrates the MPU6050 accelerometer and gyroscope offsets
 * using only the standard Arduino Wire library. Perfect for cases where
 * you can't install the I2Cdev/MPU6050 libraries.
 *
 * Hardware Setup:
 * - Arduino Nano/UNO
 * - MPU6050 connected via I2C (A4/A5)
 * - Connect VCC to 5V, GND to GND, SDA to A4, SCL to A5
 * - Place sensor on a flat, stable surface during calibration
 *
 * Usage:
 * 1. Upload this sketch to your Arduino
 * 2. Open Serial Monitor (115200 baud)
 * 3. Wait for calibration to complete (about 30 seconds)
 * 4. Note the offset values displayed
 * 5. Copy these values to your main SpaceWizard transmitter code
 *
 * Based on: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/IMU_Zero
 * Simplified for standard Arduino IDE without external libraries
 */

#include <Wire.h>

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 registers
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_PWR_MGMT_2   0x6C

// MPU6050 offset registers
#define MPU6050_XA_OFFSET_H  0x06
#define MPU6050_XA_OFFSET_L  0x07
#define MPU6050_YA_OFFSET_H  0x08
#define MPU6050_YA_OFFSET_L  0x09
#define MPU6050_ZA_OFFSET_H  0x0A
#define MPU6050_ZA_OFFSET_L  0x0B
#define MPU6050_XG_OFFSET_H  0x13
#define MPU6050_XG_OFFSET_L  0x14
#define MPU6050_YG_OFFSET_H  0x15
#define MPU6050_YG_OFFSET_L  0x16
#define MPU6050_ZG_OFFSET_H  0x17
#define MPU6050_ZG_OFFSET_L  0x18

// MPU6050 offset values (these will be calculated)
int16_t ax_offset = 0;
int16_t ay_offset = 0;
int16_t az_offset = 0;
int16_t gx_offset = 0;
int16_t gy_offset = 0;
int16_t gz_offset = 0;

// Calibration settings
#define CALIBRATION_SAMPLES 1000  // Number of samples to average
#define CALIBRATION_DELAY 3       // Delay between samples (ms)

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect

    // Initialize I2C
    Wire.begin();

    Serial.println("Simple MPU6050 Calibration Starting...");
    Serial.println("Place sensor on a flat, stable surface.");
    Serial.println("DO NOT MOVE the sensor during calibration!");
    Serial.println();

    // Initialize MPU6050
    Serial.print("Initializing MPU6050...");
    if (initializeMPU6050()) {
        Serial.println(" SUCCESS");
    } else {
        Serial.println(" FAILED");
        Serial.println("Check your MPU6050 connections and try again.");
        Serial.println("Make sure SDA is connected to A4 and SCL to A5.");
        while (1); // Halt execution
    }

    Serial.println("Calibrating accelerometer and gyroscope...");
    Serial.println("This will take approximately 30 seconds...");
    Serial.println();

    // Calibrate accelerometer
    calibrateAccelerometer();

    // Calibrate gyroscope
    calibrateGyroscope();

    // Display final offset values
    displayOffsets();

    Serial.println("Calibration complete!");
    Serial.println("Copy these offset values to your main SpaceWizard transmitter code:");
    Serial.println();

    Serial.println("// MPU6050 calibration offsets (these values are specific to your sensor)");
    Serial.println("// Run calibration first to get your sensor's values!");
    Serial.print("#define MPU_GYRO_X_OFFSET   ");
    Serial.print(gx_offset);
    Serial.println("  // Your calibrated gyro X offset");

    Serial.print("#define MPU_GYRO_Y_OFFSET   ");
    Serial.print(gy_offset);
    Serial.println("  // Your calibrated gyro Y offset");

    Serial.print("#define MPU_GYRO_Z_OFFSET   ");
    Serial.print(gz_offset);
    Serial.println("  // Your calibrated gyro Z offset");

    Serial.print("#define MPU_ACCEL_Z_OFFSET  ");
    Serial.print(az_offset);
    Serial.println("  // Your calibrated accel Z offset");

    Serial.print("#define MPU_ACCEL_X_OFFSET  ");
    Serial.print(ax_offset);
    Serial.println("  // Your calibrated accel X offset");

    Serial.print("#define MPU_ACCEL_Y_OFFSET  ");
    Serial.print(ay_offset);
    Serial.println("  // Your calibrated accel Y offset");

    Serial.println();
    Serial.println("Simply copy and paste the lines above into your transmitter code!");
    Serial.println("Upload your main transmitter code with these new offset values.");
}

void loop() {
    // Nothing to do here - calibration is complete
    delay(1000);
}

bool initializeMPU6050() {
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00); // Wake up
    if (Wire.endTransmission(true) != 0) return false;

    // Set sample rate to 1kHz
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_SMPLRT_DIV);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) return false;

    // Set gyro to ±250°/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_GYRO_CONFIG);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) return false;

    // Set accel to ±2g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_CONFIG);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) return false;

    // Test connection by reading WHO_AM_I register
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75); // WHO_AM_I register
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(MPU6050_ADDR, 1);
    if (Wire.available()) {
        uint8_t whoAmI = Wire.read();
        if (whoAmI == 0x68) { // MPU6050 should return 0x68
            return true;
        }
    }

    return false;
}

void calibrateAccelerometer() {
    Serial.println("Calibrating accelerometer...");

    long ax_sum = 0;
    long ay_sum = 0;
    long az_sum = 0;

    // Collect samples
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int16_t ax, ay, az;
        readAccelerometer(&ax, &ay, &az);

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;

        delay(CALIBRATION_DELAY);
    }

    // Calculate average offsets (sensor should read 0,0,16384 when flat)
    ax_offset = ax_sum / CALIBRATION_SAMPLES;
    ay_offset = ay_sum / CALIBRATION_SAMPLES;
    az_offset = az_sum / CALIBRATION_SAMPLES - 16384; // Z should be +1g when flat

    Serial.println("Accelerometer calibration complete.");
}

void calibrateGyroscope() {
    Serial.println("Calibrating gyroscope...");

    long gx_sum = 0;
    long gy_sum = 0;
    long gz_sum = 0;

    // Collect samples
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int16_t gx, gy, gz;
        readGyroscope(&gx, &gy, &gz);

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        delay(CALIBRATION_DELAY);
    }

    // Calculate average offsets (gyro should read 0 when stationary)
    gx_offset = gx_sum / CALIBRATION_SAMPLES;
    gy_offset = gy_sum / CALIBRATION_SAMPLES;
    gz_offset = gz_sum / CALIBRATION_SAMPLES;

    Serial.println("Gyroscope calibration complete.");
}

void readAccelerometer(int16_t *ax, int16_t *ay, int16_t *az) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 6);

    if (Wire.available() >= 6) {
        *ax = (Wire.read() << 8) | Wire.read();
        *ay = (Wire.read() << 8) | Wire.read();
        *az = (Wire.read() << 8) | Wire.read();
    }
}

void readGyroscope(int16_t *gx, int16_t *gy, int16_t *gz) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_GYRO_XOUT_H);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 6);

    if (Wire.available() >= 6) {
        *gx = (Wire.read() << 8) | Wire.read();
        *gy = (Wire.read() << 8) | Wire.read();
        *gz = (Wire.read() << 8) | Wire.read();
    }
}

void displayOffsets() {
    Serial.println("Calibration Results:");
    Serial.println("===================");

    Serial.print("Accel X Offset: ");
    Serial.println(ax_offset);

    Serial.print("Accel Y Offset: ");
    Serial.println(ay_offset);

    Serial.print("Accel Z Offset: ");
    Serial.println(az_offset);

    Serial.print("Gyro X Offset:  ");
    Serial.println(gx_offset);

    Serial.print("Gyro Y Offset:  ");
    Serial.println(gy_offset);

    Serial.print("Gyro Z Offset:  ");
    Serial.println(gz_offset);

    Serial.println();
    Serial.println("These values are specific to your MPU6050 sensor.");
    Serial.println("Copy them to your transmitter code for accurate readings.");
}
