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
 * FMT=2 AX=123 AY=45 AZ=16070 GX=-1336 GY=537 GZ=15720 YAW=45 PITCH=30 ROLL=-15 QW=16351 QX=196 QY=-721 QZ=524 IMP=0
 *
 * Where:
 * - FMT: serial format version for the Python parser
 * - AX, AY, AZ / GX, GY, GZ: raw sensor counts; the bridge preserves legacy
 *   scaling pending a coordinated physical-units correction and sensitivity retune
 * - YAW, PITCH, ROLL: orientation angles (degrees)
 * - QW, QX, QY, QZ: raw quaternion counts (divide by 16384)
 * - IMP: impact flag (0/1)
 * - SEQ, VALID: sample sequence and freshness flag from updated transmitters
 */

#include <SPI.h>
#include <RF24.h>

// ===== HARDWARE CONFIGURATION =====

// Radio Configuration
#define RADIO_CE_PIN         9
#define RADIO_CS_PIN         10
#define RADIO_PAYLOAD_SIZE   32

// Communication pipes (must match transmitter addresses)
const uint64_t RADIO_PIPES[3] = {
    0xE8E8F0F0E1LL,  // Primary communication pipe
    0xE8E8F0F0DFLL,  // Pipes 2-5 inherit pipe 1's upper address bytes
    0xE8E8F0F0E2LL
};

// Serial Configuration
#define SERIAL_BAUD_RATE     230400
#define SERIAL_FORMAT_VERSION 2

// ===== GLOBAL VARIABLES =====

// Radio communication
RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);

// Received data structure from transmitter
// Original 14 motion fields, followed by sample sequence and 0x5700|valid.
int16_t receivedData[16];
bool packetHasMetadata = false;
bool radioReady = false;
unsigned long lastPacketMs = 0;
unsigned long lastRadioAttemptMs = 0;

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
    Serial.println("Serial format: FMT=2 AX= AY= AZ= GX= GY= GZ= YAW= PITCH= ROLL= QW= QX= QY= QZ= IMP=");
}

/*
 * Initialize the nRF24L01 radio module for wireless communication.
 */
void initializeRadio() {
    lastRadioAttemptMs = millis();
    radioReady = radio.begin();
    if (!radioReady) {
        Serial.println("Radio initialization failed; retrying");
        return;
    }

    // Configure radio settings for reliable communication
    radio.enableDynamicPayloads();
    radio.setPayloadSize(RADIO_PAYLOAD_SIZE);
    radio.setDataRate(RF24_250KBPS);  // Lower data rate for better range
    radio.setAutoAck(true);           // Enable auto-acknowledgment
    radio.setCRCLength(RF24_CRC_16);  // Use 16-bit CRC for error checking

    // Retain alternate addresses for compatibility; the updated TX uses primary.
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
    unsigned long now = millis();
    if ((!radioReady || now - lastPacketMs >= 3000) && now - lastRadioAttemptMs >= 3000) {
        initializeRadio();
    }
    processRadioData();
}

/*
 * Process incoming radio data and output to serial for Python script.
 */
void processRadioData() {
    if (radioReady && radio.available()) {
        bool accepted = false;
        uint8_t packetsRead = 0;
        // Drop stale queued packets so the bridge always sees the newest pose.
        do {
            uint8_t size = radio.getDynamicPayloadSize();
            if (size != 28 && size != 32) {
                radio.flush_rx();
                break;
            }
            int16_t packet[16] = {0};
            radio.read(packet, size);
            packetsRead++;
            if (size == 32 && (packet[15] & 0xFFFE) != 0x5700) continue;
            memcpy(receivedData, packet, sizeof(receivedData));
            packetHasMetadata = (size == 32);
            accepted = true;
        } while (radio.available() && packetsRead < 8);

        // Output data in the format expected by Python script
        if (accepted) {
            lastPacketMs = millis();
            printSerialData();
        }
    }
}

/*
 * Output motion data in a compact format for the Python OSC bridge.
 * Integer serial printing is much cheaper than float-to-string formatting on AVR,
 * so the bridge parser rescales these raw counts back to engineering units.
 */
void printSerialData() {
    Serial.print("FMT=");
    Serial.print(SERIAL_FORMAT_VERSION);
    Serial.print(" AX=");
    Serial.print(receivedData[0]);
    Serial.print(" AY=");
    Serial.print(receivedData[1]);
    Serial.print(" AZ=");
    Serial.print(receivedData[2]);
    Serial.print(" GX=");
    Serial.print(receivedData[3]);
    Serial.print(" GY=");
    Serial.print(receivedData[4]);
    Serial.print(" GZ=");
    Serial.print(receivedData[5]);
    Serial.print(" YAW=");
    Serial.print(receivedData[6]);
    Serial.print(" PITCH=");
    Serial.print(receivedData[7]);
    Serial.print(" ROLL=");
    Serial.print(receivedData[8]);
    Serial.print(" QW=");
    Serial.print(receivedData[9]);
    Serial.print(" QX=");
    Serial.print(receivedData[10]);
    Serial.print(" QY=");
    Serial.print(receivedData[11]);
    Serial.print(" QZ=");
    Serial.print(receivedData[12]);
    Serial.print(" IMP=");
    Serial.print(receivedData[13] != 0 ? 1 : 0);
    if (packetHasMetadata) {
        Serial.print(" SEQ=");
        Serial.print((uint16_t)receivedData[14]);
        Serial.print(" VALID=");
        Serial.print(receivedData[15] & 1);
    }
    Serial.println();
}
