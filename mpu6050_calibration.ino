/*
 * MPU6050 Calibration Sketch for SpaceWizard Project
 *
 * This sketch calibrates the MPU6050 accelerometer and gyroscope offsets.
 * Run this sketch with your MPU6050 connected to determine the correct
 * offset values for your specific sensor, then copy these values to your
 * main SpaceWizard transmitter code.
 *
 * Hardware Setup:
 * - Arduino Nano/UNO
 * - MPU6050 connected via I2C (A4/A5)
 * - Connect VCC to 5V, GND to GND
 * - Place sensor on a flat, stable surface during calibration
 *
 * Usage:
 * 1. Upload this sketch to your Arduino
 * 2. Open Serial Monitor (115200 baud)
 * 3. Wait for calibration to complete (about 30 seconds)
 * 4. Note the offset values displayed
 * 5. Copy these values to your main transmitter code
 *
 * Based on: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/IMU_Zero
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

// MPU6050 offset values (these will be calculated)
int ax_offset = 0;
int ay_offset = 0;
int az_offset = 0;
int gx_offset = 0;
int gy_offset = 0;
int gz_offset = 0;

// Calibration settings
#define CALIBRATION_SAMPLES 1000  // Number of samples to average
#define CALIBRATION_DELAY 3       // Delay between samples (ms)

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect

    // Initialize I2C
    Wire.begin();
    TWBR = 12; // Set I2C speed to 400kHz for 16MHz CPU

    Serial.println("MPU6050 Calibration Starting...");
    Serial.println("Place sensor on a flat, stable surface.");
    Serial.println("DO NOT MOVE the sensor during calibration!");
    Serial.println();

    // Initialize MPU6050
    Serial.print("Initializing MPU6050...");
    mpu.initialize();

    if (mpu.testConnection()) {
        Serial.println(" SUCCESS");
    } else {
        Serial.println(" FAILED");
        Serial.println("Check your MPU6050 connections and try again.");
        while (1); // Halt execution
    }

    // Set MPU6050 to most sensitive range for calibration
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250°/s

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

    Serial.print("mpu.setXAccelOffset(");
    Serial.print(ax_offset);
    Serial.println(");");

    Serial.print("mpu.setYAccelOffset(");
    Serial.print(ay_offset);
    Serial.println(");");

    Serial.print("mpu.setZAccelOffset(");
    Serial.print(az_offset);
    Serial.println(");");

    Serial.print("mpu.setXGyroOffset(");
    Serial.print(gx_offset);
    Serial.println(");");

    Serial.print("mpu.setYGyroOffset(");
    Serial.print(gy_offset);
    Serial.println(");");

    Serial.print("mpu.setZGyroOffset(");
    Serial.print(gz_offset);
    Serial.println(");");

    Serial.println();
    Serial.println("Upload your main transmitter code with these new offset values.");
}

void loop() {
    // Nothing to do here - calibration is complete
    delay(1000);
}

void calibrateAccelerometer() {
    Serial.println("Calibrating accelerometer...");

    long ax_sum = 0;
    long ay_sum = 0;
    long az_sum = 0;

    // Collect samples
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int16_t ax, ay, az;
        mpu.getAcceleration(&ax, &ay, &az);

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
        mpu.getRotation(&gx, &gy, &gz);

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
