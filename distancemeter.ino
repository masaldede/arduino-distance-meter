/*
 * Arduino Distance Meter with Ultrasonic Sensor
 * 
 * Hardware Connections:
 * ---------------------
 * HC-SR04 Ultrasonic Sensor:
 *   - VCC -> 5V on Arduino
 *   - GND -> GND on Arduino
 *   - Trig -> Digital Pin 7
 *   - Echo -> Digital Pin 8
 * 
 * 16x2 LCD Display (I2C):
 *   - VCC -> 5V on Arduino
 *   - GND -> GND on Arduino
 *   - SDA -> Analog Pin A4 (SDA)
 *   - SCL -> Analog Pin A5 (SCL)
 * 
 * Push Button:
 *   - One terminal -> Digital Pin 2
 *   - Other terminal -> GND
 *   - 10k ohm pull-up resistor between Pin 2 and 5V (or use internal pull-up)
 * 
 * Optional: LED indicator
 *   - Anode (+) -> Digital Pin 13 (through 220 ohm resistor)
 *   - Cathode (-) -> GND
 */

#include <LiquidCrystal_I2C.h>  // Library for I2C LCD
#include <Wire.h>               // I2C communication library

// Pin definitions
const int TRIG_PIN = 7;         // Ultrasonic sensor trigger pin
const int ECHO_PIN = 8;         // Ultrasonic sensor echo pin
const int BUTTON_PIN = 2;       // Push button pin
const int LED_PIN = 13;         // Built-in LED pin (optional indicator)

// LCD setup - I2C address is usually 0x27 or 0x3F
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Address, columns, rows

// Variables
bool systemOn = false;          // System state (on/off)
bool lastButtonState = HIGH;    // Previous button state
bool currentButtonState = HIGH; // Current button state
unsigned long lastDebounceTime = 0;  // For button debouncing
const unsigned long debounceDelay = 50;  // Debounce delay in milliseconds
unsigned long lastMeasurement = 0;      // For measurement timing
const unsigned long measurementInterval = 200;  // Measurement interval (200ms)

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize LCD
  lcd.init();           // Initialize the LCD
  lcd.backlight();      // Turn on backlight
  
  // Display startup message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance Meter");
  lcd.setCursor(0, 1);
  lcd.print("Press to start");
  
  // Initial LED state
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("Distance Meter Initialized");
  Serial.println("Press button to toggle on/off");
}

void loop() {
  // Handle button press with debouncing
  handleButton();
  
  // If system is on, take measurements
  if (systemOn) {
    if (millis() - lastMeasurement >= measurementInterval) {
      float distance = measureDistance();
      displayDistance(distance);
      lastMeasurement = millis();
    }
  }
}

void handleButton() {
  // Read current button state
  bool reading = digitalRead(BUTTON_PIN);
  
  // Check if button state has changed (due to noise or pressing)
  if (reading != lastButtonState) {
    lastDebounceTime = millis();  // Reset debounce timer
  }
  
  // If enough time has passed since last state change
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If button state has actually changed
    if (reading != currentButtonState) {
      currentButtonState = reading;
      
      // Button was pressed (state changed from HIGH to LOW)
      if (currentButtonState == LOW) {
        toggleSystem();
      }
    }
  }
  
  lastButtonState = reading;
}

void toggleSystem() {
  systemOn = !systemOn;  // Toggle system state
  
  if (systemOn) {
    // System turned ON
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System: ON");
    lcd.setCursor(0, 1);
    lcd.print("Measuring...");
    digitalWrite(LED_PIN, HIGH);  // Turn on LED indicator
    Serial.println("System ON - Starting measurements");
  } else {
    // System turned OFF
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System: OFF");
    lcd.setCursor(0, 1);
    lcd.print("Press to start");
    digitalWrite(LED_PIN, LOW);   // Turn off LED indicator
    Serial.println("System OFF");
  }
  
  delay(500);  // Small delay to prevent multiple toggles
}

float measureDistance() {
  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse duration
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
  
  // Calculate distance in centimeters
  // Speed of sound = 343 m/s = 0.0343 cm/µs
  // Distance = (duration * 0.0343) / 2  (divided by 2 for round trip)
  float distance = (duration * 0.0343) / 2;
  
  // Handle timeout or invalid readings
  if (duration == 0 || distance > 400) {  // HC-SR04 max range ~4m
    distance = -1;  // Invalid reading
  }
  
  return distance;
}

void displayDistance(float distance) {
  lcd.setCursor(0, 0);
  lcd.print("Distance Meter  ");  // Extra spaces to clear previous text
  
  lcd.setCursor(0, 1);
  
  if (distance < 0) {
    // Invalid reading
    lcd.print("Out of range!   ");
    Serial.println("Distance: Out of range");
  } else if (distance < 2) {
    // Too close
    lcd.print("Too close!      ");
    Serial.println("Distance: Too close");
  } else {
    // Valid reading - display in cm and inches
    lcd.print(distance, 1);      // Display with 1 decimal place
    lcd.print(" cm ");
    
    // Convert to inches (1 inch = 2.54 cm)
    float inches = distance / 2.54;
    if (distance < 100) {  // If less than 100cm, show inches too
      lcd.print("(");
      lcd.print(inches, 1);
      lcd.print("\")");
    }
    
    // Clear any remaining characters
    lcd.print("    ");
    
    // Print to serial monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm (");
    Serial.print(inches);
    Serial.println(" inches)");
  }
}

/*
 * Additional Notes:
 * ----------------
 * 
 * 1. Power Requirements:
 *    - Arduino Uno: 5V via USB or 7-12V via barrel jack
 *    - HC-SR04: 5V, ~15mA
 *    - I2C LCD: 5V, ~20mA (with backlight)
 *    - Total current draw: ~50-60mA (well within Arduino limits)
 * 
 * 2. I2C LCD Address:
 *    - Common addresses: 0x27, 0x3F
 *    - Use I2C scanner sketch to find your LCD's address if needed
 * 
 * 3. Measurement Range:
 *    - HC-SR04: 2cm to 400cm
 *    - Accuracy: ±3mm
 *    - Ultrasonic frequency: 40kHz
 * 
 * 4. Troubleshooting:
 *    - If LCD doesn't display: Check I2C address and connections
 *    - If distance readings are erratic: Check sensor connections
 *    - If button doesn't respond: Check pull-up resistor or enable internal pull-up
 * 
 * 5. Possible Enhancements:
 *    - Add buzzer for proximity alarm
 *    - Store min/max readings
 *    - Add calibration function
 *    - Change units (cm/inches/feet)
 *    - Add averaging for stable readings
 */
