/*
 * ===============================================================================
 * COMPLETE ARDUINO SENSOR LIBRARY
 * ===============================================================================
 * A comprehensive library combining multiple sensors and components
 * Based on Elegoo examples and enhanced with modular functions
 *
 * Author: Arduino Community / Elegoo
 * Modified: Enhanced with modular structure and documentation
 * Date: 2025
 *
 * SUPPORTED COMPONENTS:
 * - Ultrasonic Sensor (HC-SR04)
 * - Keypad (4x4)
 * - DHT11 Temperature/Humidity Sensor
 * - Joystick
 * - IR Remote Receiver
 * - LED Matrix (MAX7219)
 * - MPU6050 Accelerometer/Gyroscope
 * - PIR Motion Sensor
 * - Photoresistor
 * - DS3231 RTC Clock
 * - Sound Sensor
 * - RFID (MFRC522)
 * - LCD Display (16x2)
 * - Shift Register (74HC595)
 * - Seven Segment Display
 * - DC Motor Control
 * - Stepper Motor
 * - RGB LED
 * - Servo Motor
 * - Active/Passive Buzzer
 *
 * USAGE:
 * 1. Uncomment the components you want to use in the setup() function
 * 2. Call the respective functions in loop() or as needed
 * 3. Adjust pin definitions as needed for your wiring
 * ===============================================================================
 */

// ===============================================================================
// LIBRARY INCLUDES
// ===============================================================================
#include "IRremote.h"        // IR receiver
#include "LedControl.h"      // LED Matrix
#include "SR04.h"            // Ultrasonic sensor
#include "pitches.h"         // Musical notes (you'll need to create this file)
#include <DS3231.h>          // RTC clock
#include <Keypad.h>          // 4x4 Keypad
#include <LiquidCrystal.h>   // LCD display
#include <MFRC522.h>         // RFID
#include <SPI.h>             // SPI communication
#include <Servo.h>           // Servo motor
#include <Stepper.h>         // Stepper motor
#include <Wire.h>            // I2C communication
#include <dht_nonblocking.h> // DHT11 sensor

// ===============================================================================
// PIN DEFINITIONS
// ===============================================================================

// Ultrasonic Sensor (HC-SR04)
#define TRIG_PIN 12
#define ECHO_PIN 11

// Keypad pins
byte rowPins[4] = {9, 8, 7, 6};
byte colPins[4] = {5, 4, 3, 2};

// DHT11 sensor
#define DHT_SENSOR_PIN 2
#define DHT_SENSOR_TYPE DHT_TYPE_11

// Joystick pins
#define SW_PIN 2
#define X_PIN 0
#define Y_PIN 1

// IR Receiver
#define IR_RECEIVER_PIN 11

// LED Matrix (MAX7219)
#define LED_MATRIX_DATA_PIN 12
#define LED_MATRIX_CLK_PIN 10
#define LED_MATRIX_CS_PIN 11

// MPU6050 I2C address
#define MPU_ADDR 0x68

// PIR Motion Sensor
#define PIR_PIN 7
#define PIR_LED_PIN 13

// Photoresistor
#define PHOTO_PIN 0

// Sound Sensor
#define SOUND_ANALOG_PIN A0
#define SOUND_DIGITAL_PIN 3
#define SOUND_LED_PIN 13

// RFID (MFRC522)
#define RFID_RST_PIN 5
#define RFID_SS_PIN 53

// LCD Display
#define LCD_RS 7
#define LCD_EN 8
#define LCD_D4 9
#define LCD_D5 10
#define LCD_D6 11
#define LCD_D7 12

// Shift Register (74HC595)
#define SHIFT_LATCH_PIN 11
#define SHIFT_CLOCK_PIN 9
#define SHIFT_DATA_PIN 12

// Seven Segment Display
#define SEVEN_SEG_LATCH_PIN 3
#define SEVEN_SEG_CLOCK_PIN 4
#define SEVEN_SEG_DATA_PIN 2

// Motor Control (L293D)
#define MOTOR_ENABLE_PIN 5
#define MOTOR_DIR_A_PIN 3
#define MOTOR_DIR_B_PIN 4

// Stepper Motor
#define STEPPER_STEPS 2048
#define STEPPER_PIN1 8
#define STEPPER_PIN2 10
#define STEPPER_PIN3 9
#define STEPPER_PIN4 11

// RGB LED
#define RGB_RED_PIN 6
#define RGB_GREEN_PIN 5
#define RGB_BLUE_PIN 3

// Servo Motor
#define SERVO_PIN 9

// Buzzer
#define BUZZER_PIN 12
#define PASSIVE_BUZZER_PIN 8

// Push Buttons
#define BUTTON_A_PIN 9
#define BUTTON_B_PIN 8
#define BUTTON_LED_PIN 5

// ===============================================================================
// GLOBAL OBJECTS AND VARIABLES
// ===============================================================================

// Component instances
SR04 ultrasonicSensor = SR04(ECHO_PIN, TRIG_PIN);
DHT_nonblocking dhtSensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
IRrecv irReceiver(IR_RECEIVER_PIN);
LedControl ledMatrix = LedControl(LED_MATRIX_DATA_PIN, LED_MATRIX_CLK_PIN, LED_MATRIX_CS_PIN, 1);
DS3231 rtcClock;
MFRC522 rfidReader(RFID_SS_PIN, RFID_RST_PIN);
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
Stepper stepperMotor(STEPPER_STEPS, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2, STEPPER_PIN4);
Servo servoMotor;

// Keypad configuration
char hexaKeys[4][4] = {
    {'1', '2', '3', 'A'}, {'4', '5', '6', 'B'}, {'7', '8', '9', 'C'}, {'*', '0', '#', 'D'}};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, 4, 4);

// Seven segment patterns
byte sevenSegDigits[10] = {
    B11111100, // 0
    B01100000, // 1
    B11011010, // 2
    B11110010, // 3
    B01100110, // 4
    B10110110, // 5
    B10111110, // 6
    B11100000, // 7
    B11111110, // 8
    B11100110  // 9
};

// Global variables
uint32_t lastIRCode = 0;
unsigned long lastSensorRead = 0;
RTCDateTime currentTime;
byte shiftRegisterLEDs = 0;

// ===============================================================================
// SETUP FUNCTION
// ===============================================================================
void setup() {
    Serial.begin(9600);
    Serial.println("Arduino Sensor Library Initialized");
    Serial.println("===================================");

    // Initialize components (uncomment as needed)
    initializeUltrasonic();
    initializeKeypad();
    initializeDHT();
    initializeJoystick();
    initializeIR();
    initializeLEDMatrix();
    initializeMPU6050();
    initializePIR();
    initializePhotoresistor();
    initializeRTC();
    initializeSoundSensor();
    initializeRFID();
    initializeLCD();
    initializeShiftRegister();
    initializeSevenSegment();
    initializeMotor();
    initializeStepper();
    initializeRGB();
    initializeServo();
    initializeBuzzer();
    initializeButtons();

    Serial.println("All components initialized!");
    delay(2000);
}

// ===============================================================================
// MAIN LOOP
// ===============================================================================
void loop() {
    // Example usage - uncomment functions you want to run

    // Sensor readings
    // readUltrasonic();
    // readKeypad();
    // readDHT();
    // readJoystick();
    // readIR();
    // readMPU6050();
    // readPIR();
    // readPhotoresistor();
    // readSoundSensor();
    // readRFID();

    // Display functions
    // updateLCD();
    // updateSevenSegment();
    // displayLEDMatrix();

    // Control functions
    // controlRGB();
    // controlServo();
    // controlStepper();
    // controlMotor();

    // Sound functions
    // playBuzzer();
    // playMelody();

    delay(100); // Small delay to prevent overwhelming the serial monitor
}

// ===============================================================================
// INITIALIZATION FUNCTIONS
// ===============================================================================

void initializeUltrasonic() {
    Serial.println("Initializing Ultrasonic Sensor...");
    // No special initialization needed for SR04 library
}

void initializeKeypad() {
    Serial.println("Initializing Keypad...");
    // No special initialization needed for Keypad library
}

void initializeDHT() {
    Serial.println("Initializing DHT11 Sensor...");
    // DHT sensor initializes automatically
}

void initializeJoystick() {
    Serial.println("Initializing Joystick...");
    pinMode(SW_PIN, INPUT);
    digitalWrite(SW_PIN, HIGH); // Enable pull-up resistor
}

void initializeIR() {
    Serial.println("Initializing IR Receiver...");
    irReceiver.enableIRIn();
}

void initializeLEDMatrix() {
    Serial.println("Initializing LED Matrix...");
    ledMatrix.shutdown(0, false);
    ledMatrix.setIntensity(0, 8);
    ledMatrix.clearDisplay(0);
}

void initializeMPU6050() {
    Serial.println("Initializing MPU6050...");
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

void initializePIR() {
    Serial.println("Initializing PIR Sensor...");
    pinMode(PIR_LED_PIN, OUTPUT);
    pinMode(PIR_PIN, INPUT);
    digitalWrite(PIR_LED_PIN, LOW);
}

void initializePhotoresistor() {
    Serial.println("Initializing Photoresistor...");
    // No special initialization needed
}

void initializeRTC() {
    Serial.println("Initializing RTC Clock...");
    rtcClock.begin();
    // Uncomment to set time manually
    // rtcClock.setDateTime(2025, 1, 1, 12, 0, 0);
}

void initializeSoundSensor() {
    Serial.println("Initializing Sound Sensor...");
    pinMode(SOUND_DIGITAL_PIN, INPUT);
    pinMode(SOUND_LED_PIN, OUTPUT);
}

void initializeRFID() {
    Serial.println("Initializing RFID Reader...");
    SPI.begin();
    rfidReader.PCD_Init();
}

void initializeLCD() {
    Serial.println("Initializing LCD Display...");
    lcd.begin(16, 2);
    lcd.print("Arduino Library");
    lcd.setCursor(0, 1);
    lcd.print("Ready!");
}

void initializeShiftRegister() {
    Serial.println("Initializing Shift Register...");
    pinMode(SHIFT_LATCH_PIN, OUTPUT);
    pinMode(SHIFT_DATA_PIN, OUTPUT);
    pinMode(SHIFT_CLOCK_PIN, OUTPUT);
}

void initializeSevenSegment() {
    Serial.println("Initializing Seven Segment Display...");
    pinMode(SEVEN_SEG_LATCH_PIN, OUTPUT);
    pinMode(SEVEN_SEG_CLOCK_PIN, OUTPUT);
    pinMode(SEVEN_SEG_DATA_PIN, OUTPUT);
}

void initializeMotor() {
    Serial.println("Initializing DC Motor...");
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_DIR_A_PIN, OUTPUT);
    pinMode(MOTOR_DIR_B_PIN, OUTPUT);
}

void initializeStepper() {
    Serial.println("Initializing Stepper Motor...");
    stepperMotor.setSpeed(15);
}

void initializeRGB() {
    Serial.println("Initializing RGB LED...");
    pinMode(RGB_RED_PIN, OUTPUT);
    pinMode(RGB_GREEN_PIN, OUTPUT);
    pinMode(RGB_BLUE_PIN, OUTPUT);
}

void initializeServo() {
    Serial.println("Initializing Servo Motor...");
    servoMotor.attach(SERVO_PIN);
}

void initializeBuzzer() {
    Serial.println("Initializing Buzzer...");
    pinMode(BUZZER_PIN, OUTPUT);
}

void initializeButtons() {
    Serial.println("Initializing Buttons...");
    pinMode(BUTTON_LED_PIN, OUTPUT);
    pinMode(BUTTON_A_PIN, INPUT_PULLUP);
    pinMode(BUTTON_B_PIN, INPUT_PULLUP);
}

// ===============================================================================
// SENSOR READING FUNCTIONS
// ===============================================================================

long readUltrasonic() {
    long distance = ultrasonicSensor.Distance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    return distance;
}

char readKeypad() {
    char key = customKeypad.getKey();
    if (key) {
        Serial.print("Key pressed: ");
        Serial.println(key);
    }
    return key;
}

bool readDHT(float *temperature, float *humidity) {
    static unsigned long measurementTimestamp = millis();

    if (millis() - measurementTimestamp > 3000ul) {
        if (dhtSensor.measure(temperature, humidity)) {
            measurementTimestamp = millis();
            Serial.print("Temperature: ");
            Serial.print(*temperature, 1);
            Serial.print("°C, Humidity: ");
            Serial.print(*humidity, 1);
            Serial.println("%");
            return true;
        }
    }
    return false;
}

void readJoystick() {
    int switchState = digitalRead(SW_PIN);
    int xValue = analogRead(X_PIN);
    int yValue = analogRead(Y_PIN);

    Serial.print("Switch: ");
    Serial.print(switchState);
    Serial.print(", X: ");
    Serial.print(xValue);
    Serial.print(", Y: ");
    Serial.println(yValue);
}

uint32_t readIR() {
    if (irReceiver.decode()) {
        if (irReceiver.decodedIRData.flags) {
            irReceiver.decodedIRData.decodedRawData = lastIRCode;
        }

        uint32_t code = irReceiver.decodedIRData.decodedRawData;
        Serial.print("IR Code: 0x");
        Serial.println(code, HEX);

        // Decode common remote buttons
        decodeIRButton(code);

        lastIRCode = code;
        irReceiver.resume();
        return code;
    }
    return 0;
}

void readMPU6050() {
    int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    Serial.print("AcX: ");
    Serial.print(AcX);
    Serial.print(" | AcY: ");
    Serial.print(AcY);
    Serial.print(" | AcZ: ");
    Serial.print(AcZ);
    Serial.print(" | Temp: ");
    Serial.print(Tmp / 340.00 + 36.53);
    Serial.print(" | GyX: ");
    Serial.print(GyX);
    Serial.print(" | GyY: ");
    Serial.print(GyY);
    Serial.print(" | GyZ: ");
    Serial.println(GyZ);
}

bool readPIR() {
    int pirValue = digitalRead(PIR_PIN);
    digitalWrite(PIR_LED_PIN, pirValue);
    if (pirValue) {
        Serial.println("Motion detected!");
    }
    return pirValue;
}

int readPhotoresistor() {
    static int historyValue = 0;
    int value = analogRead(PHOTO_PIN);

    if (abs(value - historyValue) > 10) {
        Serial.print("Light level: ");
        Serial.println(value);
        historyValue = value;
    }
    return value;
}

void readSoundSensor() {
    int analogValue = analogRead(SOUND_ANALOG_PIN);
    int digitalValue = digitalRead(SOUND_DIGITAL_PIN);

    Serial.print("Sound Level: ");
    Serial.println(analogValue);

    digitalWrite(SOUND_LED_PIN, digitalValue);
}

bool readRFID() {
    if (!rfidReader.PICC_IsNewCardPresent() || !rfidReader.PICC_ReadCardSerial()) {
        return false;
    }

    Serial.print("Card UID:");
    for (byte i = 0; i < rfidReader.uid.size; i++) {
        Serial.print(rfidReader.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(rfidReader.uid.uidByte[i], HEX);
    }
    Serial.println();

    rfidReader.PICC_HaltA();
    return true;
}

void readButtons() {
    if (digitalRead(BUTTON_A_PIN) == LOW) {
        digitalWrite(BUTTON_LED_PIN, HIGH);
        Serial.println("Button A pressed");
    }
    if (digitalRead(BUTTON_B_PIN) == LOW) {
        digitalWrite(BUTTON_LED_PIN, LOW);
        Serial.println("Button B pressed");
    }
}

// ===============================================================================
// DISPLAY AND OUTPUT FUNCTIONS
// ===============================================================================

void updateLCD() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Time: ");

    currentTime = rtcClock.getDateTime();
    lcd.print(currentTime.hour);
    lcd.print(":");
    lcd.print(currentTime.minute);

    lcd.setCursor(0, 1);
    lcd.print("Temp: ");

    float temp, humidity;
    if (readDHT(&temp, &humidity)) {
        lcd.print(temp, 1);
        lcd.print("C");
    }
}

void updateShiftRegister() {
    digitalWrite(SHIFT_LATCH_PIN, LOW);
    shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, LSBFIRST, shiftRegisterLEDs);
    digitalWrite(SHIFT_LATCH_PIN, HIGH);
}

void setShiftRegisterLED(int ledNumber, bool state) {
    if (ledNumber >= 0 && ledNumber <= 7) {
        if (state) {
            bitSet(shiftRegisterLEDs, ledNumber);
        } else {
            bitClear(shiftRegisterLEDs, ledNumber);
        }
        updateShiftRegister();
    }
}

void updateSevenSegment(int digit) {
    if (digit >= 0 && digit <= 9) {
        digitalWrite(SEVEN_SEG_LATCH_PIN, LOW);
        shiftOut(SEVEN_SEG_DATA_PIN, SEVEN_SEG_CLOCK_PIN, LSBFIRST, sevenSegDigits[digit]);
        digitalWrite(SEVEN_SEG_LATCH_PIN, HIGH);
    }
}

void displayLEDMatrix() {
    // Display "Arduino" text pattern
    byte a[5] = {B01111110, B10001000, B10001000, B10001000, B01111110};

    for (int i = 0; i < 5; i++) {
        ledMatrix.setRow(0, i, a[i]);
    }
    delay(1000);
    ledMatrix.clearDisplay(0);
}

// ===============================================================================
// MOTOR AND ACTUATOR CONTROL FUNCTIONS
// ===============================================================================

void controlMotor(int speed, bool direction) {
    analogWrite(MOTOR_ENABLE_PIN, abs(speed));

    if (direction) {
        digitalWrite(MOTOR_DIR_A_PIN, HIGH);
        digitalWrite(MOTOR_DIR_B_PIN, LOW);
    } else {
        digitalWrite(MOTOR_DIR_A_PIN, LOW);
        digitalWrite(MOTOR_DIR_B_PIN, HIGH);
    }
}

void stopMotor() {
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
}

void controlStepper(int steps) {
    stepperMotor.step(steps);
    // Turn off motor coils to save power
    digitalWrite(STEPPER_PIN1, LOW);
    digitalWrite(STEPPER_PIN2, LOW);
    digitalWrite(STEPPER_PIN3, LOW);
    digitalWrite(STEPPER_PIN4, LOW);
}

void controlServo(int angle) {
    if (angle >= 0 && angle <= 180) {
        servoMotor.write(angle);
    }
}

void sweepServo() {
    for (int angle = 0; angle <= 180; angle++) {
        servoMotor.write(angle);
        delay(10);
    }
    for (int angle = 180; angle >= 0; angle--) {
        servoMotor.write(angle);
        delay(10);
    }
}

// ===============================================================================
// RGB LED CONTROL FUNCTIONS
// ===============================================================================

void setRGBColor(int red, int green, int blue) {
    analogWrite(RGB_RED_PIN, red);
    analogWrite(RGB_GREEN_PIN, green);
    analogWrite(RGB_BLUE_PIN, blue);
}

void controlRGB() {
    // Color cycling animation
    for (int i = 0; i < 255; i++) {
        setRGBColor(255 - i, i, 0); // Red to Green
        delay(10);
    }
    for (int i = 0; i < 255; i++) {
        setRGBColor(0, 255 - i, i); // Green to Blue
        delay(10);
    }
    for (int i = 0; i < 255; i++) {
        setRGBColor(i, 0, 255 - i); // Blue to Red
        delay(10);
    }
}

// ===============================================================================
// SOUND FUNCTIONS
// ===============================================================================

void playBuzzer(int duration) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
}

void playMelody() {
    int melody[] = {262, 294, 330, 349, 392, 440, 494, 523}; // C major scale
    int duration = 500;

    for (int i = 0; i < 8; i++) {
        tone(PASSIVE_BUZZER_PIN, melody[i], duration);
        delay(600);
    }
}

void playAlarm() {
    for (int i = 0; i < 10; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }
}

// ===============================================================================
// UTILITY FUNCTIONS
// ===============================================================================

void decodeIRButton(uint32_t code) {
    switch (code) {
    case 0xBA45FF00:
        Serial.println("POWER");
        break;
    case 0xB847FF00:
        Serial.println("FUNC/STOP");
        break;
    case 0xB946FF00:
        Serial.println("VOL+");
        break;
    case 0xBB44FF00:
        Serial.println("FAST BACK");
        break;
    case 0xBF40FF00:
        Serial.println("PAUSE");
        break;
    case 0xBC43FF00:
        Serial.println("FAST FORWARD");
        break;
    case 0xF807FF00:
        Serial.println("DOWN");
        break;
    case 0xEA15FF00:
        Serial.println("VOL-");
        break;
    case 0xF609FF00:
        Serial.println("UP");
        break;
    case 0xE619FF00:
        Serial.println("EQ");
        break;
    case 0xF20DFF00:
        Serial.println("ST/REPT");
        break;
    case 0xE916FF00:
        Serial.println("0");
        break;
    case 0xF30CFF00:
        Serial.println("1");
        break;
    case 0xE718FF00:
        Serial.println("2");
        break;
    case 0xA15EFF00:
        Serial.println("3");
        break;
    case 0xF708FF00:
        Serial.println("4");
        break;
    case 0xE31CFF00:
        Serial.println("5");
        break;
    case 0xA55AFF00:
        Serial.println("6");
        break;
    case 0xBD42FF00:
        Serial.println("7");
        break;
    case 0xAD52FF00:
        Serial.println("8");
        break;
    case 0xB54AFF00:
        Serial.println("9");
        break;
    default:
        Serial.println("Unknown button");
        break;
    }
}

void printTime() {
    currentTime = rtcClock.getDateTime();
    Serial.print("Current time: ");
    Serial.print(currentTime.year);
    Serial.print("-");
    Serial.print(currentTime.month);
    Serial.print("-");
    Serial.print(currentTime.day);
    Serial.print(" ");
    Serial.print(currentTime.hour);
    Serial.print(":");
    Serial.print(currentTime.minute);
    Serial.print(":");
    Serial.println(currentTime.second);
}

void lightMeter() {
    int lightLevel = readPhotoresistor();
    int numLEDs = lightLevel / 128; // Scale to 0-8 LEDs

    shiftRegisterLEDs = 0;
    for (int i = 0; i < numLEDs; i++) {
        bitSet(shiftRegisterLEDs, i);
    }
    updateShiftRegister();
}

// ===============================================================================
// ADVANCED CONTROL FUNCTIONS
// ===============================================================================

void demoMode() {
    Serial.println("Starting Demo Mode...");

    // RGB LED demo
    Serial.println("RGB LED Demo");
    controlRGB();

    // Servo sweep demo
    Serial.println("Servo Demo");
    sweepServo();

    // Buzzer demo
    Serial.println("Buzzer Demo");
    playMelody();

    // LED Matrix demo
    Serial.println("LED Matrix Demo");
    displayLEDMatrix();

    // Seven segment counter
    Serial.println("Seven Segment Counter");
    for (int i = 0; i < 10; i++) {
        updateSevenSegment(i);
        delay(500);
    }

    Serial.println("Demo complete!");
}

void automaticLighting() {
    int lightLevel = analogRead(PHOTO_PIN);

    if (lightLevel < 300) {
        // Dark - turn on LED
        setShiftRegisterLED(0, true);
        setRGBColor(255, 255, 255); // White light
    } else {
        // Bright - turn off LED
        setShiftRegisterLED(0, false);
        setRGBColor(0, 0, 0);
    }
}

void securitySystem() {
    static bool armed = false;
    static unsigned long lastMotion = 0;

    // Check for arming/disarming via keypad
    char key = readKeypad();
    if (key == 'A') {
        armed = !armed;
        Serial.print("Security system ");
        Serial.println(armed ? "ARMED" : "DISARMED");
        playBuzzer(100);
    }

    if (armed) {
        if (readPIR()) {
            if (millis() - lastMotion > 5000) { // 5 second cooldown
                Serial.println("SECURITY ALERT: Motion detected!");
                playAlarm();
                setRGBColor(255, 0, 0); // Red alert
                lastMotion = millis();
            }
        }
    }
}

// ===============================================================================
// END OF LIBRARY
// ===============================================================================

/*
 * USAGE EXAMPLES:
 *
 * In loop(), you can call any of these functions:
 *
 * // Read sensors
 * long distance = readUltrasonic();
 * char key = readKeypad();
 * float temp, humidity;
 * bool dhtSuccess = readDHT(&temp, &humidity);
 *
 * // Control outputs
 * setRGBColor(255, 0, 0);  // Red
 * controlServo(90);         // 90 degrees
 * playBuzzer(500);         // 500ms beep
 * updateSevenSegment(5);   // Display "5"
 *
 * // Run demos
 * demoMode();              // Full demonstration
 * automaticLighting();     // Light-sensitive LED
 * securitySystem();        // Motion-activated alarm
 *
 * REQUIRED ADDITIONAL FILES:
 * - pitches.h (for musical notes)
 * - All the required libraries installed in Arduino IDE
 *
 * WIRING:
 * Follow the pin definitions at the top of this file.
 * Adjust pins as needed for your specific setup.
 */
