/*
 * SpaceWizard OSC Transmitter - MPU6050 Motion Controller for MaestroDMX OSC
 *
 * This Arduino sketch reads raw motion data from an MPU6050 accelerometer/gyroscope
 * and transmits it wirelessly using an nRF24L01 radio module. The receiver converts
 * this data to OSC messages for MaestroDMX lighting control.
 *
 * Hardware Components:
 * - Arduino Nano/UNO
 * - MPU6050 6-axis motion sensor
 * - nRF24L01 radio transceiver
 * - Push button for mode switching
 *
 * Transmits raw sensor data for OSC processing:
 * - Raw accelerometer values (16-bit signed integers)
 * - Raw gyroscope values (16-bit signed integers)
 * - Orientation angles (degrees)
 * - Impact detection flag
 * - Button state
 */

#include <SPI.h>
#include <RF24.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

// Debug configuration - comment out to disable serial output
//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)     Serial.print (x)
  #define DEBUG_PRINTDEC(x)  Serial.print (x, DEC)
  #define DEBUG_PRINTLN(x)   Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTDEC(x)
  #define DEBUG_PRINTLN(x)
#endif

// ===== HARDWARE CONFIGURATION =====

// Radio Configuration
#define RADIO_CE_PIN        9
#define RADIO_CS_PIN        10
#define RADIO_TIMEOUT_MS    3000
#define RADIO_PAYLOAD_SIZE  32  // Increased for more data

// Communication pipes (addresses for radio communication)
const uint64_t RADIO_PIPES[3] = {
  0xE8E8F0F0E1LL,  // Primary communication pipe
  0xEEFDFAF50DFLL,  // Secondary pipe
  0xEEFDFAF50E2LL   // Tertiary pipe
};

// Note: No button configuration needed for OSC version

// MPU6050 Configuration & Data
#define MPU6050_I2C_SPEED   400000  // 400kHz I2C speed

// MPU6050 calibration offsets (these values are specific to your sensor)
// Run calibration first to get your sensor's values!
#define MPU_GYRO_X_OFFSET   -465  // Your calibrated gyro X offset
#define MPU_GYRO_Y_OFFSET   90    // Your calibrated gyro Y offset
#define MPU_GYRO_Z_OFFSET   0     // Your calibrated gyro Z offset
#define MPU_ACCEL_Z_OFFSET  81    // Your calibrated accel Z offset
#define MPU_ACCEL_X_OFFSET  -230  // Your calibrated accel X offset
#define MPU_ACCEL_Y_OFFSET  -696  // Your calibrated accel Y offset

// ===== GLOBAL VARIABLES =====

// Radio communication
RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);
int16_t transmitData[10];  // [accelX, accelY, accelZ, gyroX, gyroY, gyroZ, yaw, pitch, roll, impact]

// Note: No button state management needed for OSC version

// MPU6050 sensor
MPU6050 mpu;

// Raw sensor data storage
int16_t rawAccelX = 0;      // Raw X-axis acceleration (16-bit signed)
int16_t rawAccelY = 0;      // Raw Y-axis acceleration (16-bit signed)
int16_t rawAccelZ = 0;      // Raw Z-axis acceleration (16-bit signed)
int16_t rawGyroX = 0;       // Raw X-axis gyroscope (16-bit signed)
int16_t rawGyroY = 0;       // Raw Y-axis gyroscope (16-bit signed)
int16_t rawGyroZ = 0;       // Raw Z-axis gyroscope (16-bit signed)
float yawAngle = 0.0;       // Yaw angle (degrees, 0-360)
float pitchAngle = 0.0;     // Pitch angle (degrees, -90 to +90)
float rollAngle = 0.0;      // Roll angle (degrees, -90 to +90)
bool impactDetected = false; // Impact/shock detection flag

// MPU6050 control/status variables
bool mpuInitialized = false;     // Set true if MPU6050 DMP initialization was successful
uint8_t mpuInterruptStatus;       // Holds actual interrupt status byte from MPU
uint8_t deviceStatus;            // Return status after each MPU operation (0 = success, !0 = error)
uint16_t dmpPacketSize;          // Expected DMP packet size (default is 42 bytes)
uint16_t fifoByteCount;          // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];          // FIFO storage buffer

// Motion/orientation variables for MPU6050 DMP calculations
Quaternion quaternion;           // [w, x, y, z] quaternion container for orientation
VectorFloat gravityVector;       // [x, y, z] gravity vector
float yawPitchRoll[3];           // [yaw, pitch, roll] angles in radians
VectorInt16 rawAcceleration;     // [x, y, z] raw accelerometer measurements
VectorInt16 realAcceleration;    // [x, y, z] gravity-free accelerometer measurements

// ================================================================
// ===               MPU6050 INTERRUPT HANDLER                  ===
// ================================================================

volatile bool mpuInterrupt = false;  // Indicates whether MPU interrupt pin has gone high

/*
 * Interrupt Service Routine for MPU6050 data ready signal.
 * This function is called when the MPU6050 has new data available.
 */
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      SETUP FUNCTION                      ===
// ================================================================

/*
 * Initialize all hardware components and prepare the system for operation.
 * This function runs once when the Arduino is powered on or reset.
 */
void setup() {
    initializeSerial();
    initializeMPU6050();
    initializeRadio();
}

/*
 * Initialize serial communication for debugging.
 */
void initializeSerial() {
    #ifdef DEBUG
        Serial.begin(115200);
        DEBUG_PRINTLN("SpaceWizard OSC Transmitter Starting...");
    #endif
}

// Note: No button initialization needed for OSC version

/*
 * Initialize the MPU6050 sensor and DMP (Digital Motion Processor).
 */
void initializeMPU6050() {
    // Initialize I2C communication
    Wire.begin();
    TWBR = 12;  // Set 400kHz I2C speed for 16MHz CPU

    // Initialize MPU6050
    mpu.initialize();

    // Apply calibration offsets (these values are specific to your MPU6050)
    mpu.setXGyroOffset(MPU_GYRO_X_OFFSET);
    mpu.setYGyroOffset(MPU_GYRO_Y_OFFSET);
    mpu.setZGyroOffset(MPU_GYRO_Z_OFFSET);
    mpu.setXAccelOffset(MPU_ACCEL_X_OFFSET);
    mpu.setYAccelOffset(MPU_ACCEL_Y_OFFSET);
    mpu.setZAccelOffset(MPU_ACCEL_Z_OFFSET);

    // Initialize DMP
    deviceStatus = mpu.dmpInitialize();

    if (deviceStatus == 0) {
        // DMP initialization successful
        mpu.setDMPEnabled(true);

        // Enable interrupt detection on pin 2
        attachInterrupt(0, dmpDataReady, RISING);
        mpuInterruptStatus = mpu.getIntStatus();

        // Mark MPU as ready for use
        mpuInitialized = true;

        // Get expected DMP packet size for FIFO operations
        dmpPacketSize = mpu.dmpGetFIFOPacketSize();

        DEBUG_PRINTLN("MPU6050 DMP initialized successfully");
    } else {
        DEBUG_PRINT("MPU6050 DMP initialization failed: ");
        DEBUG_PRINTLN(deviceStatus);
    }
}

/*
 * Initialize the nRF24L01 radio module for wireless communication.
 */
void initializeRadio() {
    radio.begin();

    // Configure radio settings for optimal performance
    radio.enableDynamicPayloads();
    radio.setPayloadSize(RADIO_PAYLOAD_SIZE);
    radio.setDataRate(RF24_250KBPS);  // Lower data rate for better range
    radio.setAutoAck(false);          // Disable auto-acknowledgment for simplicity
    radio.setRetries(2, 15);          // Set retransmission parameters
    radio.setCRCLength(RF24_CRC_16);  // Use 16-bit CRC for error checking

    // Open primary communication pipe
    radio.openWritingPipe(RADIO_PIPES[0]);
    radio.powerUp();

    DEBUG_PRINTLN("Radio initialized successfully");
}

// ================================================================
// ===                      MAIN LOOP                          ===
// ================================================================

/*
 * Main program loop that runs continuously after setup.
 * Handles button input, MPU6050 data processing, and radio transmission.
 */
void loop() {
    processMPU6050Data();
    transmitMotionData();
}

// Note: No button handling needed for OSC version

/*
 * Process MPU6050 data and prepare it for transmission.
 */
void processMPU6050Data() {
    if (!mpuInitialized) return;  // Skip if MPU6050 not initialized

    // Wait for MPU interrupt or check if data is available
    mpuInterrupt = false;
    mpuInterruptStatus = mpu.getIntStatus();
    fifoByteCount = mpu.getFIFOCount();

    // Handle FIFO overflow
    if ((mpuInterruptStatus & 0x10) || fifoByteCount == 1024) {
        mpu.resetFIFO();
        DEBUG_PRINTLN("MPU6050 FIFO overflow - reset");
    }
    // Process available data packets
    else if (mpuInterruptStatus & 0x02) {
        // Read all available packets from FIFO
        while (mpu.getFIFOCount() > dmpPacketSize) {
            mpu.getFIFOBytes(fifoBuffer, dmpPacketSize);
        }

        // Process the latest motion data
        mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
        mpu.dmpGetAccel(&rawAcceleration, fifoBuffer);
        mpu.dmpGetGravity(&gravityVector, &quaternion);
        mpu.dmpGetLinearAccel(&realAcceleration, &rawAcceleration, &gravityVector);
        mpu.dmpGetYawPitchRoll(yawPitchRoll, &quaternion, &gravityVector);

        // Store raw sensor data
        rawAccelX = rawAcceleration.x;
        rawAccelY = rawAcceleration.y;
        rawAccelZ = rawAcceleration.z;

        // Get raw gyroscope data
        mpu.getRotation(&rawGyroX, &rawGyroY, &rawGyroZ);

        // Convert orientation angles to degrees
        yawAngle = yawPitchRoll[0] * 180 / M_PI;
        if (yawAngle < 0) yawAngle += 360;  // Ensure positive range

        pitchAngle = yawPitchRoll[1] * 180 / M_PI;
        rollAngle = yawPitchRoll[2] * 180 / M_PI;

        // Simple impact detection based on linear acceleration magnitude
        float accelMagnitude = sqrt(pow(realAcceleration.x, 2) +
                                  pow(realAcceleration.y, 2) +
                                  pow(realAcceleration.z, 2));
        impactDetected = (accelMagnitude > 2000);  // Threshold for impact detection
    }
}

/*
 * Prepare and transmit motion data via radio.
 * Data format: [accelX, accelY, accelZ, gyroX, gyroY, gyroZ, yaw, pitch, roll, impact]
 */
void transmitMotionData() {
    // Prepare data packet with all required values (no button state for OSC version)
    transmitData[0] = rawAccelX;     // Raw X-axis acceleration (16-bit signed)
    transmitData[1] = rawAccelY;     // Raw Y-axis acceleration (16-bit signed)
    transmitData[2] = rawAccelZ;     // Raw Z-axis acceleration (16-bit signed)
    transmitData[3] = rawGyroX;      // Raw X-axis gyroscope (16-bit signed)
    transmitData[4] = rawGyroY;      // Raw Y-axis gyroscope (16-bit signed)
    transmitData[5] = rawGyroZ;      // Raw Z-axis gyroscope (16-bit signed)
    transmitData[6] = (int16_t)yawAngle;     // Yaw angle (degrees, 0-360)
    transmitData[7] = (int16_t)pitchAngle;   // Pitch angle (degrees, -90 to +90)
    transmitData[8] = (int16_t)rollAngle;    // Roll angle (degrees, -90 to +90)
    transmitData[9] = impactDetected ? 1 : 0; // Impact detection flag

    // Transmit data on all three pipes for redundancy
    radio.openWritingPipe(RADIO_PIPES[0]);
    radio.write(transmitData, sizeof(transmitData));

    radio.openWritingPipe(RADIO_PIPES[1]);
    radio.write(transmitData, sizeof(transmitData));

    radio.openWritingPipe(RADIO_PIPES[2]);
    radio.write(transmitData, sizeof(transmitData));

    // Debug output
    #ifdef DEBUG
        static int debugCounter = 0;
        if (debugCounter % 20 == 0) {  // Print every 20 transmissions
            DEBUG_PRINT("Accel: ");
            DEBUG_PRINT(rawAccelX); DEBUG_PRINT(",");
            DEBUG_PRINT(rawAccelY); DEBUG_PRINT(",");
            DEBUG_PRINT(rawAccelZ); DEBUG_PRINT(" ");

            DEBUG_PRINT("Gyro: ");
            DEBUG_PRINT(rawGyroX); DEBUG_PRINT(",");
            DEBUG_PRINT(rawGyroY); DEBUG_PRINT(",");
            DEBUG_PRINT(rawGyroZ); DEBUG_PRINT(" ");

            DEBUG_PRINT("Angles: ");
            DEBUG_PRINT(yawAngle);
            DEBUG_PRINT(",");
            DEBUG_PRINT(pitchAngle);
            DEBUG_PRINT(",");
            DEBUG_PRINT(rollAngle);
            DEBUG_PRINT(" ");

            DEBUG_PRINT("Impact: ");
            DEBUG_PRINTLN(impactDetected);
        }
        debugCounter++;
    #endif
}
