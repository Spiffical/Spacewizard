/*
 * SpaceWizard OSC Receiver - Serial Bridge for MaestroDMX Control
 *
 * This Arduino sketch receives motion data from a transmitter unit via nRF24L01
 * radio and outputs it over USB serial for a Python script to convert to OSC
 * messages for MaestroDMX lighting control.
 *
 * Hardware Components:
 * - Arduino Nano/UNO
 * - nRF24L01 radio transceiver
 *
 * Serial Output Format (one line per packet):
 * AX=0.12 AY=0.03 AZ=0.98 GX=-10.2 GY=4.1 GZ=120.0 YAW=45.0 PITCH=30.0 ROLL=-15.0 IMP=0
 *
 * Where:
 * - AX, AY, AZ: accelerometer values (g-force)
 * - GX, GY, GZ: gyroscope values (degrees/second)
 * - YAW, PITCH, ROLL: orientation angles (degrees)
 * - IMP: impact flag (0/1)
 */

#include <SPI.h>
#include <RF24.h>

// ===== HARDWARE CONFIGURATION =====

// Radio Configuration
#define RADIO_CE_PIN         9
#define RADIO_CS_PIN         10
#define RADIO_PAYLOAD_SIZE   16

// Communication pipes (must match transmitter addresses)
const uint64_t RADIO_PIPES[3] = {
    0xE8E8F0F0E1LL,  // Primary communication pipe
    0xEEFDFAF50DFLL,  // Secondary communication pipe
    0xEEFDFAF50E2LL   // Tertiary communication pipe
};

// Serial Configuration
#define SERIAL_BAUD_RATE     115200

// ===== GLOBAL VARIABLES =====

// Radio communication
RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);

// Received data structure from transmitter
// [accelX, accelY, accelZ, gyroX, gyroY, gyroZ, yaw, pitch, roll, impact]
int16_t receivedData[10];

// Motion data (properly scaled from received values)
float accelX = 0.0;        // X-axis acceleration (g-force, -16g to +16g)
float accelY = 0.0;        // Y-axis acceleration (g-force, -16g to +16g)
float accelZ = 0.0;        // Z-axis acceleration (g-force, -16g to +16g)
float gyroX = 0.0;         // X-axis rotation (degrees/second, -2000 to +2000)
float gyroY = 0.0;         // Y-axis rotation (degrees/second, -2000 to +2000)
float gyroZ = 0.0;         // Z-axis rotation (degrees/second, -2000 to +2000)
float yawAngle = 0.0;      // Yaw angle (degrees, 0-360)
float pitchAngle = 0.0;    // Pitch angle (degrees, -90 to +90)
float rollAngle = 0.0;     // Roll angle (degrees, -90 to +90)
bool impactDetected = false;  // Impact/shock detection flag

// ================================================================
// ===                      SETUP FUNCTION                      ===
// ================================================================

/*
 * Initialize all hardware components and prepare the system for operation.
 */
void setup() {
    initializeSerial();
    initializeRadio();
}

/*
 * Initialize serial communication for data output to Python script.
 */
void initializeSerial() {
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("SpaceWizard OSC Receiver Starting...");
    Serial.println("Serial format: AX= AY= AZ= GX= GY= GZ= IMP=");
}

/*
 * Initialize the nRF24L01 radio module for wireless communication.
 */
void initializeRadio() {
    radio.begin();

    // Configure radio settings for reliable communication
    radio.enableDynamicPayloads();
    radio.setPayloadSize(RADIO_PAYLOAD_SIZE);
    radio.setDataRate(RF24_250KBPS);  // Lower data rate for better range
    radio.setAutoAck(true);           // Enable auto-acknowledgment
    radio.setCRCLength(RF24_CRC_16);  // Use 16-bit CRC for error checking

    // Open communication pipes for reading (try all three for redundancy)
    radio.openReadingPipe(1, RADIO_PIPES[0]);
    radio.openReadingPipe(2, RADIO_PIPES[1]);
    radio.openReadingPipe(3, RADIO_PIPES[2]);
    radio.startListening();

    Serial.println("Radio initialized successfully");
}

// ================================================================
// ===                      MAIN LOOP                          ===
// ================================================================

/*
 * Main program loop that runs continuously after setup.
 * Handles radio communication and serial data output.
 */
void loop() {
    processRadioData();
}

/*
 * Process incoming radio data and output to serial for Python script.
 */
void processRadioData() {
    if (radio.available()) {
        // Read motion data from transmitter
        radio.read(receivedData, sizeof(receivedData));

        // Convert received integer values to meaningful units
        convertReceivedData();

        // Output data in the format expected by Python script
        printSerialData();

        // Small delay to prevent overwhelming the serial output
        delay(25);  // ~40 Hz update rate
    }
}

/*
 * Convert raw received data to proper units for OSC mapping.
 * Expected data structure: [accelX, accelY, accelZ, gyroX, gyroY, gyroZ, yaw, pitch, roll, impact]
 */
void convertReceivedData() {
    // Raw accelerometer data (16-bit signed integers from MPU6050)
    // Scale to g-force using MPU6050 sensitivity (16384 LSB/g for ±2g range)
    accelX = receivedData[0] / 16384.0;  // Convert to g-force (-2g to +2g)
    accelY = receivedData[1] / 16384.0;  // Convert to g-force (-2g to +2g)
    accelZ = receivedData[2] / 16384.0;  // Convert to g-force (-2g to +2g)

    // Gyroscope data (degrees/second)
    // Scale using MPU6050 sensitivity (131 LSB/°/s for ±250°/s range)
    gyroX = receivedData[3] / 131.0;     // Convert to °/s (-250 to +250)
    gyroY = receivedData[4] / 131.0;     // Convert to °/s (-250 to +250)
    gyroZ = receivedData[5] / 131.0;     // Convert to °/s (-250 to +250)

    // Orientation angles (from DMP calculations)
    // These are already in degrees, just need proper range conversion
    yawAngle = receivedData[6];          // 0-360 degrees
    if (yawAngle < 0) yawAngle += 360;   // Ensure positive range

    pitchAngle = receivedData[7];        // -90 to +90 degrees
    rollAngle = receivedData[8];         // -90 to +90 degrees

    // Impact detection (boolean flag from transmitter)
    impactDetected = (receivedData[9] != 0);
}

/*
 * Output motion data in the format expected by the Python OSC bridge.
 * Format: AX=0.12 AY=0.03 AZ=0.98 GX=-10.2 GY=4.1 GZ=120.0 YAW=45.0 PITCH=30.0 ROLL=-15.0 IMP=0
 */
void printSerialData() {
    Serial.print("AX=");
    Serial.print(accelX, 4);      // 4 decimal places for precision
    Serial.print(" AY=");
    Serial.print(accelY, 4);
    Serial.print(" AZ=");
    Serial.print(accelZ, 4);
    Serial.print(" GX=");
    Serial.print(gyroX, 2);       // 2 decimal places for gyro
    Serial.print(" GY=");
    Serial.print(gyroY, 2);
    Serial.print(" GZ=");
    Serial.print(gyroZ, 2);
    Serial.print(" YAW=");
    Serial.print(yawAngle, 2);
    Serial.print(" PITCH=");
    Serial.print(pitchAngle, 2);
    Serial.print(" ROLL=");
    Serial.print(rollAngle, 2);
    Serial.print(" IMP=");
    Serial.print(impactDetected ? 1 : 0);
    Serial.println();
}
