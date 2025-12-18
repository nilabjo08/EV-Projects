// Include the Wire library for I2C communication
#include <Wire.h>
// Include the Liquid Crystal I2C library for LCD display
#include <LiquidCrystal_I2C.h>

// --- Pin Definitions (Confirmed to match your Tinkercad diagram) ---
const int voltageSensorPin = A0; // Analog pin connected to Voltage Potentiometer
const int tempSensorPin = A1;    // Analog pin connected to TMP36 Temperature Sensor

const int greenLedPin = 7;       // Digital pin for Green "OK" LED
const int redLedPin = 5;         // Digital pin for Red "Low Battery" LED
const int yellowLedPin = 6;      // Digital pin for Yellow "Overheat" LED
const int buzzerPin = 4;         // Digital pin for the Buzzer

// --- LCD Setup ---
// Define the LCD object with your confirmed address 0x27, 16 characters, 2 rows.
// This is compatible with your pcf8574 based I2C LCD.
LiquidCrystal_I2C lcd(0x27, 16, 2); // Address: 0x27, 16 characters, 2 rows

// --- Calibration and Thresholds ---

// Voltage Sensor Conversion (for Potentiometer simulating higher voltage):
// Your potentiometer outputs 0-5V to A0.
// If you want to simulate a 0-20V range, then 5V at A0 corresponds to 20V actual.
// So, the scaling factor is 20V / 5V = 4.0.
// Adjust this 'VOLTAGE_SCALING_FACTOR' if your desired simulated voltage range changes.
const float VOLTAGE_SCALING_FACTOR = 4.0; 

// TMP36 Temperature Sensor Conversion:
// TMP36 outputs 10mV per degree Celsius, with a 500mV offset at 0C.
// Arduino's analogRead gives 0-1023 for 0-5V (0-5000mV).
const float ARDUINO_REF_VOLTAGE_MV = 5000.0; // Arduino's 5V reference in mV
const float ADC_RESOLUTION = 1024.0;    // 10-bit ADC (0-1023 values)
const float TMP36_MV_PER_DEGREE = 10.0; // 10mV per degree C
const float TMP36_OFFSET_MV = 500.0;    // 500mV output at 0C for TMP36


// --- Thresholds for Status ---
const float LOW_BATTERY_THRESHOLD = 9.3; // Volts (below this is low battery)
const float OVERHEAT_TEMP_THRESHOLD = 50.0; // Degrees Celsius (above this is overheat)


void setup() {
    Serial.begin(9600); // Initialize serial communication for debugging

    // Initialize the I2C LCD
    Wire.begin(); // Explicitly start I2C communication (good practice)
    lcd.init();      // Initialize the LCD controller
    delay(100); // Small delay after init can sometimes help
    lcd.backlight(); // Turn on the backlight (now confirmed working visually!)

    lcd.clear(); // Clear any initial junk
    lcd.setCursor(0,0);
    lcd.print("System Monitor");
    lcd.setCursor(0,1);
    lcd.print("Ready...");
    delay(2000); // Display initial message for 2 seconds
    lcd.clear(); // Clear the screen after initial message

    // Set LED pins as outputs
    pinMode(greenLedPin, OUTPUT);
    pinMode(redLedPin, OUTPUT);
    pinMode(yellowLedPin, OUTPUT);
    pinMode(buzzerPin, OUTPUT);

    // Initial state: all outputs off
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(buzzerPin, LOW);
}

void loop() {
    // --- Read Analog Values & Convert ---

    // Read Voltage Sensor (Potentiometer)
    int rawVoltage = analogRead(voltageSensorPin);
    // Convert raw analog value to voltage at Arduino pin (0-5V), then scale to actual simulated voltage.
    float measuredVoltage = (rawVoltage / ADC_RESOLUTION) * (ARDUINO_REF_VOLTAGE_MV / 1000.0) * VOLTAGE_SCALING_FACTOR;

    // Read Temperature Sensor (TMP36)
    int rawTemp = analogRead(tempSensorPin);
    // Convert raw analog value to mV at pin, then apply TMP36 formula to get Celsius.
    float voltageAtTempPinMv = rawTemp * (ARDUINO_REF_VOLTAGE_MV / ADC_RESOLUTION);
    float temperatureC = (voltageAtTempPinMv - TMP36_OFFSET_MV) / TMP36_MV_PER_DEGREE;


    // --- Status Handling & Display ---
    String statusMessage1 = ""; // First line of LCD
    String statusMessage2 = ""; // Second line of LCD

    // --- IMPORTANT: Reset ALL LEDs and Buzzer before determining new state ---
    // This ensures only the correct LED for the current state is ON.
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    noTone(buzzerPin); // Ensure buzzer is off if no active tone is commanded

    // --- Decision Logic (Prioritized according to project requirements) ---

    // 1. Check for OVERHEAT (Highest Priority)
    if (temperatureC >= OVERHEAT_TEMP_THRESHOLD) {
        statusMessage1 = "OVERHEAT!";
        statusMessage2 = "Temp: " + String(temperatureC, 1) + "C";
        digitalWrite(yellowLedPin, HIGH); // *** YELLOW LED ON for Overheat ***
        tone(buzzerPin, 1000); // Continuous 1KHz tone
    }
    // 2. Else, if NOT Overheating, check for LOW BATTERY
    else if (measuredVoltage <= LOW_BATTERY_THRESHOLD) {
        statusMessage1 = "LOW BATTERY!";
        statusMessage2 = "Volt: " + String(measuredVoltage, 1) + "V";
        digitalWrite(redLedPin, HIGH); // *** RED LED ON for Low Battery ***
        tone(buzzerPin, 500); // Continuous 500Hz tone
    }
    // 3. Else, if NEITHER Overheat nor Low Battery, then it's BATTERY OK
    else {
        statusMessage1 = "BATTERY OK";
        statusMessage2 = "Temp: " + String(temperatureC, 1) + "C";
        digitalWrite(greenLedPin, HIGH); // *** GREEN LED ON for Battery OK ***
        // No tone for OK state, buzzer remains OFF due to noTone() at start of loop
        // Also show voltage on second line if everything is OK
        statusMessage2 += " Volt: " + String(measuredVoltage, 1) + "V";
    }

    // --- Display on LCD and Serial Monitor ---

    // Clear LCD for new messages and display current status
    // (This clear happens once per second due to delay(1000) below)
    lcd.clear();
    lcd.setCursor(0, 0); // Column 0, Row 0
    lcd.print(statusMessage1);
    lcd.setCursor(0, 1); // Column 0, Row 1
    lcd.print(statusMessage2);

    // Print detailed readings to Serial Monitor for debugging
    Serial.print("Raw Voltage: ");
    Serial.print(rawVoltage);
    Serial.print(" -> Measured Voltage: ");
    Serial.print(measuredVoltage, 2); // Print with 2 decimal places
    Serial.println(" V");

    Serial.print("Raw Temp: ");
    Serial.print(rawTemp);
    Serial.print(" -> Temperature: ");
    Serial.print(temperatureC, 1); // Print with 1 decimal place
    Serial.println(" C");

    Serial.println("Status: " + statusMessage1);
    Serial.println("--------------------");

    delay(1000); // Wait for 1 second before next reading
}