# Arduino Distance Meter

A digital distance measuring device using ultrasonic sensor with LCD display and push button control.

## Features

- 📏 Accurate distance measurement (2cm - 400cm range)
- 🖥️ Real-time display on 16x2 LCD screen
- 🔘 Simple push button on/off control
- 💡 LED status indicator
- 📊 Dual unit display (centimeters and inches)
- 🔧 Serial monitor output for debugging
- ⚡ Low power consumption

## Hardware Requirements

### Components List
| Component | Quantity | Description |
|-----------|----------|-------------|
| Arduino Uno Rev3 | 1 | Main microcontroller |
| HC-SR04 Ultrasonic Sensor | 1 | Distance measurement sensor |
| 16x2 I2C LCD Display | 1 | Display module |
| Push Button | 1 | System on/off control |
| 10kΩ Resistor | 1 | Pull-up resistor for button (optional) |
| 220Ω Resistor | 1 | LED current limiting (optional) |
| Breadboard | 1 | For prototyping |
| Jumper Wires | ~10 | Male-to-male connections |

### Pin Connections

#### HC-SR04 Ultrasonic Sensor
```
HC-SR04    →    Arduino Uno
VCC        →    5V
GND        →    GND
Trig       →    Digital Pin 7
Echo       →    Digital Pin 8
```

#### 16x2 I2C LCD Display
```
LCD (I2C)  →    Arduino Uno
VCC        →    5V
GND        →    GND
SDA        →    Analog Pin A4
SCL        →    Analog Pin A5
```

#### Push Button
```
Button     →    Arduino Uno
Terminal 1 →    Digital Pin 2
Terminal 2 →    GND
```

#### Status LED (Optional)
```
LED        →    Arduino Uno
Anode (+)  →    Digital Pin 13 (through 220Ω resistor)
Cathode(-) →    GND
```

## Software Requirements

### Arduino IDE Setup
1. Download and install [Arduino IDE](https://www.arduino.cc/en/software)
2. Install required library:
   - Go to **Sketch** → **Include Library** → **Manage Libraries**
   - Search for "LiquidCrystal_I2C"
   - Install the library by Frank de Brabander

### Library Dependencies
- `LiquidCrystal_I2C.h` - For I2C LCD control
- `Wire.h` - For I2C communication (built-in)

## Installation

1. **Hardware Assembly:**
   - Connect components according to pin diagram above
   - Double-check all connections
   - Ensure proper power supply (5V for all components)

2. **Software Upload:**
   - Open Arduino IDE
   - Copy the provided code
   - Save as `distance_meter.ino`
   - Select **Board**: Arduino Uno
   - Select correct **Port**
   - Click **Upload**

3. **Testing:**
   - Open Serial Monitor (Tools → Serial Monitor)
   - Set baud rate to 9600
   - Press the button to start measurements

## Usage Instructions

1. **Power On**: Connect Arduino to USB or power supply
2. **System Start**: Press the push button to turn on the system
3. **Measuring**: Point the ultrasonic sensor toward target object
4. **Reading**: Distance appears on LCD in cm and inches
5. **System Off**: Press button again to turn off

### Display Information
- **Line 1**: "Distance Meter" (when measuring)
- **Line 2**: Distance reading with units
- **Special Messages**:
  - "Out of range!" - Object too far (>400cm)
  - "Too close!" - Object too near (<2cm)
  - "System: OFF" - When system is turned off

## Technical Specifications

| Parameter | Value |
|-----------|-------|
| Measurement Range | 2cm - 400cm |
| Accuracy | ±3mm |
| Resolution | 0.1cm |
| Measurement Frequency | 5 readings/second |
| Operating Voltage | 5V DC |
| Current Consumption | ~60mA |
| Operating Temperature | 0°C to 70°C |
| Ultrasonic Frequency | 40kHz |

## Troubleshooting

### Common Issues

#### LCD Not Displaying
- **Check**: I2C address (try 0x27 or 0x3F)
- **Check**: SDA/SCL connections
- **Check**: Power supply to LCD

#### Erratic Distance Readings
- **Check**: Ultrasonic sensor connections
- **Check**: Target surface (avoid soft/angled surfaces)
- **Check**: Interference from other ultrasonic devices

#### Button Not Responding
- **Check**: Button connections
- **Check**: Pull-up resistor (or enable internal pull-up)
- **Check**: Debounce delay settings

#### No Serial Output
- **Check**: Baud rate set to 9600
- **Check**: USB connection
- **Check**: Serial Monitor settings

### I2C LCD Address Detection
If LCD doesn't work, find the correct I2C address:
```cpp
// I2C Scanner Code
#include <Wire.h>
void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("I2C Scanner");
}
void loop() {
  for(byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if(Wire.endTransmission() == 0) {
      Serial.print("Found I2C: 0x");
      Serial.println(i, HEX);
    }
  }
  delay(1000);
}
```

## Enhancements & Modifications

### Possible Upgrades
- **Buzzer Alarm**: Add proximity warning
- **Data Logging**: Store measurements to SD card
- **WiFi Connectivity**: Send data to web server
- **Multiple Units**: Switch between cm/inches/feet
- **Calibration**: User calibration function
- **Min/Max Memory**: Store extreme readings
- **Averaging**: Smooth readings with moving average

### Code Modifications
- Change measurement interval: Modify `measurementInterval` variable
- Adjust debounce delay: Modify `debounceDelay` variable
- Change LCD address: Modify I2C address in LCD initialization

## Circuit Diagram

```
    Arduino Uno Rev3
         ┌─────┐
    5V ──┤     ├── GND
         │     │
    A4 ──┤ SDA ├── A5 (SCL)
         │     │
    D2 ──┤ BTN ├── D7 (TRIG)
         │     │
   D13 ──┤ LED ├── D8 (ECHO)
         └─────┘
           │
      [HC-SR04]
         │ │ │ │
         V G T E
         C C R C
         C N R H
           D I O
             G
```

## License

This project is open source. Feel free to modify and distribute.

## Author

Created for Arduino Uno Rev3 with educational and practical applications in mind. 

## Version History

- **v1.0** - Initial release with basic functionality
- Current features: Distance measurement, LCD display, button control

---

**Note**: Always double-check connections before powering on. Incorrect wiring may damage components.
