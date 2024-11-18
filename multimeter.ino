#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include "max6675.h" // Include MAX6675 library

LiquidCrystal_PCF8574 lcd(0x27); // Adjust your I2C address if necessary

// Resistance Measurement Variables
const int refResistor = 3300; // Reference resistor value (in ohms)
const int analogPinRes = A2;  // Pin for resistance measurement
const float Vcc = 5.0;        // Adjust this if your Arduino's 5V pin is not exactly 5V

// Current Measurement Variables
const int sensorPin = A4;     // Analog pin for ACS712
const float sensitivity = 0.185; // Sensitivity for ACS712-5A
float VREF = 5;
const int numReadings = 7;

// Voltage Measurement Variables
const int voltagePin = A0;      // Pin for voltage measurement
const int continuityPin = A2;   // Pin for continuity check
const int continuityThreshold = 750;
const int voltageReadingsCount = 10;
float continuityEMA = 0.0;
float voltageEMA = 0.0;
const float continuityAlpha = 0.1;
const float voltageAlpha = 0.1;

// Temperature Measurement Variables
int soPin = 12;   // SO=Serial Out
int csPin = 10;   // CS = Chip Select
int sckPin = 13;  // SCK = Serial Clock
MAX6675 temperatureSensor(sckPin, csPin, soPin); // Create instance object for MAX6675

// User Input Variable
int mode = 0; // 1 for Resistance, 2 for Current, 3 for Voltage, 4 for Temperature

void setup() {
    Serial.begin(9600);
    lcd.begin(16, 2);
    lcd.setBacklight(1);
    lcd.clear();
    lcd.print("Select Mode:");
    lcd.setCursor(0, 1);
    lcd.print("1:R 2:I 3:V 4:T");
}

void loop() {
    if (Serial.available()) {
        char input = Serial.read();
        if (input == '1') mode = 1; // Resistance Mode
        if (input == '2') mode = 2; // Current Mode
        if (input == '3') mode = 3; // Voltage Mode
        if (input == '4') mode = 4; // Temperature Mode
        lcd.clear();
    }

    switch (mode) {
        case 1:
            measureResistance();
            break;
        case 2:
            measureCurrent();
            break;
        case 3:
            measureVoltage();
            break;
        case 4:
            measureTemperature();
            break;
        default:
            lcd.setCursor(0, 0);
            lcd.print("Select Mode:");
            lcd.setCursor(0, 1);
            lcd.print("1:R 2:I 3:V 4:T");
            break;
    }

    delay(1000); // Refresh rate
}

// Function to measure resistance
void measureResistance() {
    int sensorValue = analogRead(analogPinRes);
    float voltage = sensorValue * (Vcc / 1023.0);
    float unknownResistance = 0;

    lcd.setCursor(0, 0);
    lcd.print("Resistance:     ");
    if (voltage >= Vcc - 0.1) {
        lcd.setCursor(0, 1);
        lcd.print("Out of Range   ");
    } else if (voltage <= 0.1) {
        lcd.setCursor(0, 1);
        lcd.print("Short Circuit  ");
    } else {
        unknownResistance = (voltage * refResistor) / (Vcc - voltage);
        lcd.setCursor(0, 1);
        lcd.print(unknownResistance);
        Serial.println(unknownResistance);
        lcd.print(" ohms       ");
    }
}

// Function to measure current
float getMedianReading() {
    int readings[numReadings];
    for (int i = 0; i < numReadings; i++) {
        readings[i] = analogRead(sensorPin);
        delay(10);
    }
    for (int i = 0; i < numReadings - 1; i++) {
        for (int j = i + 1; j < numReadings; j++) {
            if (readings[i] > readings[j]) {
                int temp = readings[i];
                readings[i] = readings[j];
                readings[j] = temp;
            }
        }
    }
    return readings[numReadings / 2];
}

void measureCurrent() {
    float sensorValue = getMedianReading();
    float voltage = sensorValue * (5.0 / 1023.0);
    float current = (VREF - voltage);

    lcd.setCursor(0, 0);
    lcd.print("Current:        ");
    lcd.setCursor(0, 1);
    lcd.print(current);
    Serial.println(current);
    lcd.print(" A          ");
}

// Function to measure voltage
void measureVoltage() {
    int continuityValue = analogRead(continuityPin);
    continuityEMA = continuityAlpha * continuityValue + (1.0 - continuityAlpha) * continuityEMA;
    bool isContinuous = (continuityEMA < continuityThreshold);

    lcd.setCursor(0, 0);
    if (isContinuous) {
        lcd.print("Continuity: YES");
        float voltageSum = 0.0;
        for (int i = 0; i < voltageReadingsCount; i++) {
            int sensorValue = analogRead(voltagePin);
            float currentVout = sensorValue * (5.0 / 1023.0) * (3333.0 / 33.0);
            voltageEMA = voltageAlpha * currentVout + (1.0 - voltageAlpha) * voltageEMA;
            voltageSum += currentVout;
            delay(10);
        }
        float avgVoltage = voltageSum / 10.0;
        lcd.setCursor(0, 1);
        lcd.print("Voltage: ");
        lcd.print(avgVoltage, 2);
        lcd.print(" V ");
    } else {
        lcd.setCursor(0, 1);
        lcd.print("Voltage: NA    ");
    }
}

// Function to measure temperature
void measureTemperature() {
    float celsius = temperatureSensor.readCelsius();
    float fahrenheit = temperatureSensor.readFahrenheit();

    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(celsius);
    lcd.print(" C ");

    lcd.setCursor(0, 1);
    lcd.print(fahrenheit);
    lcd.print(" F ");
    
    Serial.print("Temperature: ");
    Serial.print(celsius);
    Serial.print(" °C, ");
    Serial.print(fahrenheit);
    Serial.println(" °F");
}