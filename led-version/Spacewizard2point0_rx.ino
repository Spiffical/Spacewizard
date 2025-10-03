/*
 * SpaceWizard Receiver - NeoPixel LED Controller with nRF24L01 Wireless
 *
 * This Arduino sketch receives motion data from a transmitter unit via nRF24L01
 * radio and controls NeoPixel LED strips based on the received yaw, pitch, roll,
 * and button state information. Multiple LED animation patterns are supported.
 *
 * Hardware Components:
 * - Arduino Nano/UNO
 * - nRF24L01 radio transceiver
 * - NeoPixel LED strip (40 LEDs)
 * - Optional: Serial connection for debugging
 *
 * The receiver interprets motion data to create various LED lighting effects:
 * - Scanner patterns that follow device orientation
 * - Color cycling based on tilt angles
 * - Multiple animation modes controlled by button states
 */

#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

// ===== HARDWARE CONFIGURATION =====

// NeoPixel LED strip configuration
#define NEOPIXEL_PIN        8
#define NUM_LEDS           40
#define LED_TYPE          NEO_GRB + NEO_KHZ800

// Radio communication configuration
#define RADIO_CE_PIN         9
#define RADIO_CS_PIN         10
#define RADIO_PAYLOAD_SIZE   16
#define RECEIVER_TIMEOUT_MS  5000  // Turn off LEDs if no signal for 5 seconds

// Communication pipes (must match transmitter addresses)
const uint64_t RADIO_PIPES[2] = {
    0xE8E8F0F0E1LL,  // Primary communication pipe
    0xEEFDFAF50DFLL   // Secondary communication pipe
};

// ===== GLOBAL VARIABLES =====

// Motion data received from transmitter
uint16_t receivedData[6];  // [yaw, pitch, roll, holdMode, pressMode, accelZ]
int yawAngle = 0;          // Yaw angle in degrees (0-360)
int pitchAngle = 0;        // Pitch angle in degrees (0-180)
int rollAngle = 0;         // Roll angle in degrees (0-180)
int holdMode = 0;          // 0 = normal mode, 1 = hold mode (different LED patterns)
int pressMode = 0;         // 0-4 different press modes for pattern variations
int accelerationZ = 0;     // Z-axis acceleration for impact detection

// Timing and signal management
unsigned long lastSignalTime = 0;  // Timestamp of last received signal
int accelHistory[3] = {0, 0, 0};   // Store recent acceleration values for filtering

// LED control variables
int smoothedPitch = 0;     // Filtered pitch value for smoother transitions
int previousPitchValue = 0; // Previous pitch value for smoothing
int currentPitchValue = 0;  // Current pitch value before smoothing

// ===== LED ANIMATION SYSTEM =====

// Animation pattern types
enum AnimationPattern {
    PATTERN_NONE,           // No animation
    PATTERN_RAINBOW_CYCLE,  // Cycling rainbow colors
    PATTERN_THEATER_CHASE,  // Theater-style chasing lights
    PATTERN_COLOR_WIPE,     // Wiping color across strip
    PATTERN_SCANNER,       // Back-and-forth scanning effect
    PATTERN_FADE,          // Color fading transitions
    PATTERN_SCANNER_MAN,   // Manual position scanner (motion controlled)
    PATTERN_FULLCOLOR_MAN, // Full strip color based on motion
    PATTERN_VARCOLOR_MAN   // Variable color sections based on motion
};

// Animation direction options
enum AnimationDirection {
    DIRECTION_FORWARD,      // Animation moves forward
    DIRECTION_REVERSE       // Animation moves backward
};

// NeoPixel LED strip instance
Adafruit_NeoPixel ledStrip(NUM_LEDS, NEOPIXEL_PIN, LED_TYPE);

/*
 * Enhanced NeoPixel pattern controller class.
 * Extends Adafruit_NeoPixel with animation pattern management.
 */
class NeoPatterns : public Adafruit_NeoPixel {
    public:
        // Animation state management
        AnimationPattern activePattern;    // Currently running animation pattern
        AnimationDirection direction;     // Direction of animation movement

        // Timing control
        unsigned long animationInterval;  // Milliseconds between animation updates
        unsigned long lastUpdateTime;     // Timestamp of last animation update

        // Color and pattern parameters
        uint32_t primaryColor;            // Primary color for animations
        uint32_t secondaryColor;          // Secondary color for animations
        uint16_t totalSteps;              // Total steps in current animation
        uint16_t currentStep;             // Current position in animation

        // Animation completion callback
        void (*onCompleteCallback)();     // Function to call when animation completes

        /*
         * Constructor - initialize the NeoPixel strip with animation support.
         */
        NeoPatterns(uint16_t pixelCount, uint8_t pin, uint8_t type, void (*callback)() = NULL)
            : Adafruit_NeoPixel(pixelCount, pin, type) {
            onCompleteCallback = callback;
        }
    
        /*
         * Update the current animation pattern.
         * Should be called regularly from the main loop.
         */
        void Update() {
            if (holdMode == 0) {
                // Manual control patterns (motion-responsive)
                switch(activePattern) {
                    case PATTERN_SCANNER_MAN:
                        ScannerManUpdate();
                        break;
                    case PATTERN_FULLCOLOR_MAN:
                        FullColorManUpdate();
                        break;
                    case PATTERN_VARCOLOR_MAN:
                        VarColorManUpdate();
                        break;
                    default:
                        break;
                }
            }
            else if (holdMode == 1) {
                // Automated patterns (timed animations)
                if ((millis() - lastUpdateTime) > animationInterval) {
                    lastUpdateTime = millis();

                    switch(activePattern) {
                        case PATTERN_RAINBOW_CYCLE:
                            RainbowCycleUpdate();
                            break;
                        case PATTERN_THEATER_CHASE:
                            TheaterChaseUpdate();
                            break;
                        case PATTERN_COLOR_WIPE:
                            ColorWipeUpdate();
                            break;
                        case PATTERN_SCANNER:
                            ScannerUpdate();
                            break;
                        case PATTERN_FADE:
                            FadeUpdate();
                            break;
                        default:
                            break;
                    }
                }
            }
        }
	
        /*
         * Advance the animation by one step and handle wraparound.
         */
        void Increment() {
            if (direction == DIRECTION_FORWARD) {
                currentStep++;
                if (currentStep >= totalSteps) {
                    currentStep = 0;
                    if (onCompleteCallback != NULL) {
                        onCompleteCallback();  // Call completion callback
                    }
                }
            }
            else {  // DIRECTION_REVERSE
                currentStep--;
                if (currentStep <= 0) {
                    currentStep = totalSteps - 1;
                    if (onCompleteCallback != NULL) {
                        onCompleteCallback();  // Call completion callback
                    }
                }
            }
        }

        /*
         * Reverse the direction of the current animation.
         */
        void Reverse() {
            if (direction == DIRECTION_FORWARD) {
                direction = DIRECTION_REVERSE;
                currentStep = totalSteps - 1;
            }
            else {
                direction = DIRECTION_FORWARD;
                currentStep = 0;
            }
        }
    
        /*
         * Initialize manual scanner pattern (motion-controlled position).
         */
        void ScannerMan(uint32_t color) {
            activePattern = PATTERN_SCANNER_MAN;
            primaryColor = color;
        }

        /*
         * Update manual scanner pattern based on current roll angle.
         */
        void ScannerManUpdate() {
            // Map roll angle to LED position (-30 to 80 degrees -> 0 to NUM_LEDS)
            rollAngle = map(constrain(rollAngle, -30, 80), -30, 80, 0, numPixels());

            for (int i = 0; i < numPixels(); i++) {
                if (i == rollAngle) {  // Active scanner position
                    setPixelColor(i, primaryColor);
                }
                else {  // Fading tail effect
                    setPixelColor(i, DimColor(getPixelColor(i)));
                }
            }
            show();
        }

        /*
         * Initialize full color manual pattern (motion-controlled hue).
         */
        void FullColorMan() {
            activePattern = PATTERN_FULLCOLOR_MAN;
        }

        /*
         * Update full color pattern based on pitch angle.
         */
        void FullColorManUpdate() {
            // Map pitch to color wheel position (0-90 degrees -> 0-255 color wheel)
            smoothedPitch = map(constrain(smoothedPitch, 0, 90), 0, 90, 0, 255);

            for (uint16_t i = 0; i < numPixels(); i++) {
                setPixelColor(i, Wheel((i + smoothedPitch) & 255));
            }
            show();
        }

        /*
         * Initialize variable color manual pattern.
         */
        void VarColorMan() {
            activePattern = PATTERN_VARCOLOR_MAN;
        }

        /*
         * Update variable color pattern based on roll and pitch.
         */
        void VarColorManUpdate() {
            // Map roll angle to active LED count (0-80 degrees -> 0 to NUM_LEDS)
            rollAngle = map(constrain(rollAngle, 0, 80), 0, 80, 0, numPixels());
            // Map pitch to color wheel position (0-90 degrees -> 0-255)
            pitchAngle = map(constrain(pitchAngle, 0, 90), 0, 90, 0, 255);

            for (int i = 0; i < numPixels(); i++) {
                if (i <= rollAngle) {
                    setPixelColor(i, Wheel(i + pitchAngle));
                }
                else {
                    setPixelColor(i, 0);  // Turn off LEDs beyond roll position
                }
            }
            show();
        }
    
        /*
         * Initialize rainbow cycle pattern.
         */
        void RainbowCycle(uint8_t interval, AnimationDirection dir = DIRECTION_FORWARD) {
            activePattern = PATTERN_RAINBOW_CYCLE;
            animationInterval = interval;
            totalSteps = 255;
            currentStep = 0;
            direction = dir;
        }

        /*
         * Update rainbow cycle pattern (first 3 LEDs only for demo).
         */
        void RainbowCycleUpdate() {
            for (int i = 0; i < 3; i++) {
                setPixelColor(i, Wheel(((i * 256 / 3) + currentStep) & 255));
            }
            show();
            Increment();
        }
 
        /*
         * Initialize theater chase pattern.
         */
        void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval, AnimationDirection dir = DIRECTION_FORWARD) {
            activePattern = PATTERN_THEATER_CHASE;
            animationInterval = interval;
            totalSteps = numPixels();
            primaryColor = color1;
            secondaryColor = color2;
            currentStep = 0;
            direction = dir;
        }

        /*
         * Update theater chase pattern (every 3rd LED alternating colors).
         */
        void TheaterChaseUpdate() {
            for (int i = 0; i < numPixels(); i++) {
                if ((i + currentStep) % 3 == 0) {
                    setPixelColor(i, primaryColor);
                }
                else {
                    setPixelColor(i, secondaryColor);
                }
            }
            show();
            Increment();
        }
 
        /*
         * Initialize color wipe pattern.
         */
        void ColorWipe(uint32_t color, uint8_t interval, AnimationDirection dir = DIRECTION_FORWARD) {
            activePattern = PATTERN_COLOR_WIPE;
            animationInterval = interval;
            totalSteps = numPixels();
            primaryColor = color;
            currentStep = 0;
            direction = dir;
        }

        /*
         * Update color wipe pattern (progressive fill).
         */
        void ColorWipeUpdate() {
            setPixelColor(currentStep, primaryColor);
            show();
            Increment();
        }

        /*
         * Initialize scanner pattern (back and forth movement).
         */
        void Scanner(uint32_t color, uint8_t interval) {
            activePattern = PATTERN_SCANNER;
            animationInterval = interval;
            totalSteps = (numPixels() - 1) * 2;  // Round trip distance
            primaryColor = color;
            currentStep = 0;
        }

        /*
         * Update scanner pattern (moving light with fading tail).
         */
        void ScannerUpdate() {
            for (int i = 0; i < numPixels(); i++) {
                if (i == currentStep) {  // Forward scanner position
                    setPixelColor(i, primaryColor);
                }
                else if (i == totalSteps - currentStep) {  // Reverse scanner position
                    setPixelColor(i, primaryColor);
                }
                else {  // Fading tail effect
                    setPixelColor(i, DimColor(getPixelColor(i)));
                }
            }
            show();
            Increment();
        }
    
        /*
         * Initialize fade pattern (smooth color transitions).
         */
        void Fade(uint32_t color1, uint32_t color2, uint16_t steps, uint8_t interval, AnimationDirection dir = DIRECTION_FORWARD) {
            activePattern = PATTERN_FADE;
            animationInterval = interval;
            totalSteps = steps;
            primaryColor = color1;
            secondaryColor = color2;
            currentStep = 0;
            direction = dir;
        }

        /*
         * Update fade pattern (smooth color interpolation).
         */
        void FadeUpdate() {
            // Calculate linear interpolation between colors
            uint8_t red = ((Red(primaryColor) * (totalSteps - currentStep)) + (Red(secondaryColor) * currentStep)) / totalSteps;
            uint8_t green = ((Green(primaryColor) * (totalSteps - currentStep)) + (Green(secondaryColor) * currentStep)) / totalSteps;
            uint8_t blue = ((Blue(primaryColor) * (totalSteps - currentStep)) + (Blue(secondaryColor) * currentStep)) / totalSteps;

            ColorSet(Color(red, green, blue));
            show();
            Increment();
        }
   
        /*
         * Calculate 50% dimmed version of a color (for fading tails).
         */
        uint32_t DimColor(uint32_t color) {
            // Shift RGB components right by 1 bit (divide by 2)
            return Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
        }

        /*
         * Set all pixels to a single color synchronously.
         */
        void ColorSet(uint32_t color) {
            for (int i = 0; i < numPixels(); i++) {
                setPixelColor(i, color);
            }
            show();
        }

        /*
         * Extract red component from 32-bit color value.
         */
        uint8_t Red(uint32_t color) {
            return (color >> 16) & 0xFF;
        }

        /*
         * Extract green component from 32-bit color value.
         */
        uint8_t Green(uint32_t color) {
            return (color >> 8) & 0xFF;
        }

        /*
         * Extract blue component from 32-bit color value.
         */
        uint8_t Blue(uint32_t color) {
            return color & 0xFF;
        }
    
        /*
         * Generate color wheel value (0-255) for rainbow effects.
         * Creates smooth color transitions: red -> green -> blue -> red.
         */
        uint32_t Wheel(byte position) {
            position = 255 - position;
            if (position < 85) {
                return Color(255 - position * 3, 0, position * 3);
            }
            else if (position < 170) {
                position -= 85;
                return Color(0, position * 3, 255 - position * 3);
            }
            else {
                position -= 170;
                return Color(position * 3, 255 - position * 3, 0);
            }
        }
};
 
// Create NeoPixel controller instance
NeoPatterns ledController(NUM_LEDS, NEOPIXEL_PIN, LED_TYPE, NULL);

// ================================================================
// ===                      SETUP FUNCTION                      ===
// ================================================================

/*
 * Initialize all hardware components and prepare the system for operation.
 * This function runs once when the Arduino is powered on or reset.
 */
void setup() {
    initializeSerial();
    initializeLEDs();
    initializeRadio();
}

/*
 * Initialize serial communication for debugging.
 */
void initializeSerial() {
    Serial.begin(57600);
    Serial.println("SpaceWizard Receiver Starting...");
}

/*
 * Initialize NeoPixel LED strip.
 */
void initializeLEDs() {
    ledController.begin();
    ledController.show();  // Initialize all pixels to 'off'
    Serial.println("NeoPixel LEDs initialized");
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

    // Open primary communication pipe for reading
    radio.openReadingPipe(1, RADIO_PIPES[0]);
    radio.startListening();

    radio.powerUp();

    Serial.println("Radio initialized successfully");
}
 
// ================================================================
// ===                      MAIN LOOP                          ===
// ================================================================

/*
 * Main program loop that runs continuously after setup.
 * Handles radio communication, data processing, and LED control.
 */
void loop() {
    checkSignalTimeout();
    processRadioData();
    updateLEDPatterns();
}

/*
 * Check if we've lost radio signal and turn off LEDs if timeout exceeded.
 */
void checkSignalTimeout() {
    if (millis() - lastSignalTime > RECEIVER_TIMEOUT_MS) {
        ledController.setBrightness(0);
        ledController.show();
    }
}

/*
 * Process incoming radio data and update motion variables.
 */
void processRadioData() {
    if (radio.available()) {
        lastSignalTime = millis();  // Reset timeout timer

        // Read motion data from transmitter
        radio.read(receivedData, sizeof(receivedData));

        // Extract motion angles and control states
        yawAngle = receivedData[0];
        pitchAngle = receivedData[1];
        rollAngle = receivedData[2];
        holdMode = receivedData[3];
        pressMode = receivedData[4];
        accelerationZ = receivedData[5];

        // Update acceleration history for impact detection
        accelHistory[2] = accelHistory[1];
        accelHistory[1] = accelHistory[0];
        accelHistory[0] = accelerationZ;

        // Smooth pitch values for better LED transitions
        currentPitchValue = pitchAngle;
        smoothedPitch = (previousPitchValue + currentPitchValue) / 2.0;
        previousPitchValue = currentPitchValue;

        // Debug output (uncomment if needed)
        // Serial.println(rollAngle);
    }
}

/*
 * Update LED patterns based on current motion data and control modes.
 */
void updateLEDPatterns() {
    if (holdMode == 0) {
        // Manual control patterns (motion-responsive)
        ledController.Update();

        switch (pressMode) {
            case 0:
                // Scanner pattern - red light follows roll angle
                ledController.activePattern = PATTERN_SCANNER_MAN;
                ledController.setBrightness(175);
                ledController.ScannerMan(ledController.Color(255, 0, 0));
                break;

            case 1:
                // Full color pattern - brightness controlled by acceleration impacts
                ledController.activePattern = PATTERN_FULLCOLOR_MAN;
                ledController.FullColorMan();

                // Detect sharp acceleration changes for impact effects
                if (detectImpact()) {
                    ledController.setBrightness(255);
                } else {
                    ledController.setBrightness(0);
                }
                break;

            case 2:
                // Variable color pattern - combines roll and pitch
                ledController.activePattern = PATTERN_VARCOLOR_MAN;
                ledController.setBrightness(175);
                ledController.VarColorMan();
                break;
        }
    }
    else if (holdMode == 1) {
        // Automated patterns (timed animations)
        ledController.Update();

        switch (pressMode) {
            case 0:
                // Theater chase pattern
                ledController.TheaterChase(
                    ledController.Color(255, 255, 0),  // Yellow
                    ledController.Color(0, 0, 50),     // Dark blue
                    100                                // 100ms interval
                );
                break;

            case 1:
                // Rainbow cycle pattern
                ledController.RainbowCycle(10);  // 10ms interval
                break;

            case 2:
                // Scanner pattern
                ledController.Scanner(
                    ledController.Color(255, 0, 0),  // Red
                    55                               // 55ms interval
                );
                break;
        }
    }
}

/*
 * Detect sharp acceleration changes for impact effects.
 * Returns true if an impact is detected based on acceleration history.
 */
bool detectImpact() {
    // Check for direction change with significant magnitude
    bool directionChange = (accelHistory[0] > 0 && accelHistory[2] < 0) ||
                          (accelHistory[0] < 0 && accelHistory[2] > 0);

    bool sufficientMagnitude = abs(accelHistory[0] - accelHistory[2]) > 2000;
    bool minimumThreshold = abs(accelHistory[0]) > 200;

    return directionChange && sufficientMagnitude && minimumThreshold;
}
