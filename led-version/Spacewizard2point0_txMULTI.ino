/*
 * SpaceWizard Transmitter - MPU6050 Motion Controller with nRF24L01 Wireless
 *
 * This Arduino sketch reads motion data from an MPU6050 accelerometer/gyroscope
 * and transmits it wirelessly using an nRF24L01 radio module. The device also
 * includes button controls for different operating modes.
 *
 * Hardware Components:
 * - Arduino Nano/UNO
 * - MPU6050 6-axis motion sensor
 * - nRF24L01 radio transceiver
 * - Push button for mode switching
 *
 * The transmitter sends yaw, pitch, roll angles along with button states
 * to a receiver unit that controls LED patterns based on the motion data.
 */

#include <SPI.h>
#include <RF24.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

// Debug configuration - comment out to disable serial output
#define DEBUG

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
#define RADIO_PAYLOAD_SIZE  16

// Communication pipes (addresses for radio communication)
const uint64_t RADIO_PIPES[3] = {
  0xE8E8F0F0E1LL,  // Primary communication pipe
  0xEEFDFAF50DFLL,  // Secondary pipe
  0xEEFDFAF50E2LL   // Tertiary pipe
};

// Button Configuration
#define BUTTON_PIN          6
#define BUTTON_DEBOUNCE_MS  20
#define BUTTON_HOLD_MS      2000

// MPU6050 Configuration & Data
#define MPU6050_I2C_SPEED   400000  // 400kHz I2C speed

// MPU6050 calibration offsets (these values are specific to your sensor)
#define MPU_GYRO_X_OFFSET   93
#define MPU_GYRO_Y_OFFSET   21
#define MPU_GYRO_Z_OFFSET   -31
#define MPU_ACCEL_Z_OFFSET  692
#define MPU_ACCEL_X_OFFSET  -1449
#define MPU_ACCEL_Y_OFFSET  2673

// ===== GLOBAL VARIABLES =====

// Radio communication
RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);
uint16_t transmitData[6];  // Array to hold data being transmitted

// Button state management
int buttonValue = 0;        // Current button state (HIGH/LOW)
int lastButtonValue = 0;    // Previous button state for edge detection
long buttonPressTime = 0;   // Timestamp when button was pressed
long buttonReleaseTime = 0; // Timestamp when button was released
bool ignoreButtonUp = false; // Flag to ignore button release after hold
int holdMode = 0;           // 0 = normal mode, 1 = hold mode activated
int pressMode = 0;          // 0-4 different press modes

// MPU6050 sensor
MPU6050 mpu;

// Motion data storage
int accelHistory[3] = {0, 0, 0};  // Store recent acceleration values
int currentAccelZ = 0;           // Current Z-axis acceleration

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
VectorInt16 worldAcceleration;   // [x, y, z] world-frame accelerometer measurements

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
    initializeButton();
    initializeMPU6050();
    initializeRadio();
}

/*
 * Initialize serial communication for debugging.
 */
void initializeSerial() {
    #ifdef DEBUG
        Serial.begin(9600);
        DEBUG_PRINTLN("SpaceWizard Transmitter Starting...");
    #endif
}

/*
 * Initialize the button pin with pull-up resistor.
 */
void initializeButton() {
    pinMode(BUTTON_PIN, INPUT);
    digitalWrite(BUTTON_PIN, HIGH);  // Enable pull-up resistor
}

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
    handleButtonInput();
    processMPU6050Data();
    transmitMotionData();
}

/*
 * Handle button press and release events to change operating modes.
 */
void handleButtonInput() {
    buttonValue = digitalRead(BUTTON_PIN);

    // Check for button press
    if (buttonValue == LOW && lastButtonValue == HIGH &&
        (millis() - buttonReleaseTime) > BUTTON_DEBOUNCE_MS) {
        buttonPressTime = millis();
    }

    // Check for button release
    if (buttonValue == HIGH && lastButtonValue == LOW &&
        (millis() - buttonPressTime) > BUTTON_DEBOUNCE_MS) {
        if (!ignoreButtonUp) {
            handleButtonPress();
        } else {
            ignoreButtonUp = false;
        }
        buttonReleaseTime = millis();
    }

    // Check for button hold (long press)
    if (buttonValue == LOW && (millis() - buttonPressTime) > BUTTON_HOLD_MS) {
        handleButtonHold();
        ignoreButtonUp = true;
        buttonPressTime = millis();
    }

    lastButtonValue = buttonValue;
}

/*
 * Handle short button press events (mode cycling).
 */
void handleButtonPress() {
    pressMode = (pressMode + 1) % 5;  // Cycle through 5 press modes (0-4)
}

/*
 * Handle long button hold events (hold mode toggle).
 */
void handleButtonHold() {
    holdMode = !holdMode;  // Toggle between normal and hold mode
    pressMode = 0;        // Reset press mode when hold mode changes
}

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
        int packetsAvailable = fifoByteCount / dmpPacketSize;

        // Read all available packets from FIFO
        while (mpu.getFIFOCount() > dmpPacketSize) {
            mpu.getFIFOBytes(fifoBuffer, dmpPacketSize);
        }

        // Process the latest motion data
        mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
        mpu.dmpGetAccel(&rawAcceleration, fifoBuffer);
        mpu.dmpGetGravity(&gravityVector, &quaternion);
        mpu.dmpGetLinearAccel(&realAcceleration, &rawAcceleration, &gravityVector);
        mpu.dmpGetLinearAccelInWorld(&worldAcceleration, &realAcceleration, &quaternion);
        mpu.dmpGetYawPitchRoll(yawPitchRoll, &quaternion, &gravityVector);

        // Update acceleration history for filtering
        accelHistory[2] = accelHistory[1];
        accelHistory[1] = accelHistory[0];
        accelHistory[0] = worldAcceleration.z;
    }
}

/*
 * Prepare and transmit motion data via radio.
 */
void transmitMotionData() {
    // Prepare data packet [yaw, pitch, roll, holdMode, pressMode, accelZ]
    transmitData[0] = abs(yawPitchRoll[0] * 180 / M_PI);  // Convert yaw to degrees
    transmitData[1] = abs(yawPitchRoll[1] * 180 / M_PI);  // Convert pitch to degrees
    transmitData[2] = abs(yawPitchRoll[2] * 180 / M_PI);  // Convert roll to degrees
    transmitData[3] = holdMode;
    transmitData[4] = pressMode;
    transmitData[5] = accelHistory[0];

    // Transmit data on all three pipes for redundancy
    radio.openWritingPipe(RADIO_PIPES[0]);
    radio.write(transmitData, sizeof(transmitData));

    radio.openWritingPipe(RADIO_PIPES[1]);
    radio.write(transmitData, sizeof(transmitData));

    radio.openWritingPipe(RADIO_PIPES[2]);
    radio.write(transmitData, sizeof(transmitData));
}
