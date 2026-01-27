# Smart Drain System 🌊

An automated intelligent drainage system built with Arduino that detects and manages water flow, debris, and blockages using multiple sensors and actuators.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Configuration](#configuration)
- [How It Works](#how-it-works)
- [Threshold Tuning](#threshold-tuning)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## 🔍 Overview

The Smart Drain System is an IoT-based solution designed to automatically manage drainage systems in environments prone to flooding, debris accumulation, and blockages. The system uses multiple sensors to monitor water levels, detect metal objects, measure turbidity, and track rainfall, while automated motors and servos physically manage the drainage flow.

## ✨ Features

### Water Level Monitoring
- **Dual-level detection**: Monitors both low and high water levels
- **Blockage detection**: Identifies severe and partial water blockages
- **Overflow prevention**: Alerts when water levels become critical

### Debris Management
- **Ultrasonic object detection**: Detects objects within 20cm
- **Automated conveyor system**: Automatically removes detected debris
- **Configurable run time**: 5-second operation cycle per detection

### Metal Detection
- **Active metal sensing**: Detects metallic objects in the drain
- **Servo-controlled sorting**: Automatically diverts metal debris
- **Latch mechanism**: 800ms hold time to stabilize detection

### Water Quality Monitoring
- **Turbidity sensor**: Monitors water cleanliness
- **Real-time status**: Reports dirty vs. clean water conditions

### Automated Cleaning
- **Wiper system**: Periodic cleaning mechanism
- **Adjustable cycle**: Customizable ON/OFF timing
- **Variable speed control**: PWM-controlled motor speed

### Rain Detection
- **Rain sensor integration**: Monitors precipitation
- **Blockage correlation**: Helps identify drainage issues during rainfall

### Diagnostic Features
- **Serial monitoring**: 10-second interval status updates
- **Comprehensive logging**: All sensor readings and system states
- **Time-based alerts**: Blockage detection with timing windows

## 🛠 Hardware Requirements

### Microcontroller
- **Arduino Mega 2560** (required for sufficient I/O pins)

### Sensors
- **Water Level Sensors** (x2)
  - Low level sensor → A0
  - High level sensor → A3
- **Turbidity Sensor** → A1
- **Rain Sensor** → A2
- **Ultrasonic Sensor (HC-SR04)**
  - Trigger → Pin 37
  - Echo → Pin 38
- **Metal Detector Module** → Pin 36 (Digital)

### Actuators
- **DC Motor with L298N Driver** (x2)
  - Wiper motor (EN: Pin 5, IN1: Pin 30, IN2: Pin 31)
  - Conveyor motor (EN: Pin 6, IN1: Pin 32, IN2: Pin 33)
- **Servo Motor** (SG90 or similar) → Pin 9

### Power Supply
- **12V DC supply** for motors (via L298N)
- **5V DC supply** for Arduino and sensors
- **External power recommended** for stable motor operation

### Additional Components
- Jumper wires
- Breadboard (optional, for prototyping)
- Motor mounting brackets
- Waterproof enclosure (recommended)

## 📌 Pin Configuration

### Analog Pins
```
A0 → Water Low Level Sensor
A1 → Turbidity Sensor
A2 → Rain Sensor
A3 → Water High Level Sensor
```

### Digital Pins
```
Pin 5  → Wiper Motor Enable (PWM)
Pin 6  → Conveyor Motor Enable (PWM)
Pin 9  → Metal Sorting Servo
Pin 30 → Wiper Motor IN1
Pin 31 → Wiper Motor IN2
Pin 32 → Conveyor Motor IN1
Pin 33 → Conveyor Motor IN2
Pin 36 → Metal Detector Input
Pin 37 → Ultrasonic Trigger
Pin 38 → Ultrasonic Echo
```

## 🏗 System Architecture

### Sensing Layer
1. **Water Level Management**
   - Monitors low and high water thresholds
   - Calculates blockage based on time and level correlation
   - Differentiates between partial and severe blockages

2. **Object Detection**
   - Ultrasonic distance measurement (up to 20cm)
   - Latch mechanism prevents false triggers
   - Automatic conveyor activation

3. **Metal Detection**
   - Digital signal processing with pull-up resistor
   - 800ms latch window for stability
   - Servo-controlled diversion mechanism

4. **Environmental Monitoring**
   - Turbidity analysis for water quality
   - Rain detection for predictive maintenance

### Control Layer
1. **Conveyor Motor Control**
   - Trigger: Object detected within 20cm
   - Action: Run forward for 5 seconds at speed 145/255
   - Post-action: Automatic stop and reset

2. **Wiper Motor Control**
   - Cyclic operation: 300ms ON, 2000ms OFF
   - Speed: 40/255 (slow cleaning action)
   - Continuous operation for maintenance

3. **Metal Servo Control**
   - Neutral position: 90°
   - Detection position: 20°
   - Automatic return after latch expires

### Monitoring Layer
- Serial output every 10 seconds
- Real-time sensor values
- System state reporting
- Blockage alerts

## 📥 Installation

### 1. Hardware Setup
```
1. Connect all sensors according to the pin configuration
2. Wire both L298N motor drivers to Arduino and motors
3. Attach servo motor to pin 9
4. Ensure proper power distribution (separate power for motors)
5. Test individual components before full integration
```

### 2. Software Setup
```bash
# Clone the repository
git clone https://github.com/Robiulawal527/smartDrain.git
cd smartDrain

# Open the Arduino IDE
# File → Open → Select smartDrainSystem.ino

# Select your board
# Tools → Board → Arduino Mega 2560

# Select your port
# Tools → Port → (Your Arduino port)

# Upload the sketch
# Sketch → Upload
```

### 3. Verification
```
1. Open Serial Monitor (Tools → Serial Monitor)
2. Set baud rate to 9600
3. Verify sensor readings appear every 10 seconds
4. Test each subsystem individually
```

## ⚙️ Configuration

### Adjustable Thresholds

```cpp
// Water Level Thresholds (0-1023)
#define WATER_LOW_TH       500   // Increase if too sensitive
#define WATER_HIGH_TH      650   // Adjust for overflow detection

// Water Quality
#define TURB_DIRTY_TH      600   // Lower = more sensitive to dirt

// Rain Detection
#define RAIN_WET_TH        500   // Adjust based on sensor

// Object Detection
#define DETECT_CM          20    // Maximum detection distance (cm)
```

### Motor Speed Settings

```cpp
// Conveyor Speed (0-255)
#define CONV_SPEED         145   // Adjust for optimal debris removal

// Wiper Speed
const int WIPER_SPEED = 40;      // Lower = slower cleaning
```

### Timing Parameters

```cpp
// Conveyor Run Time
#define MOTOR_RUN_MS       5000UL   // Milliseconds per cycle

// Wiper Cycle
const unsigned long WIPER_OFF_TIME = 2000UL;  // OFF duration (ms)
const unsigned long WIPER_ON_TIME  = 300UL;   // ON duration (ms)

// Metal Detection Latch
const unsigned long METAL_LATCH_MS = 800UL;   // Stabilization time

// Blockage Detection Window
const unsigned long BLOCKAGE_TIME = 15000UL;  // 15 seconds
```

### Monitoring Interval

```cpp
#define PRINT_EVERY_MS     10000UL  // Serial output frequency
```

## 🔄 How It Works

### Normal Operation Flow

1. **System Initialization**
   ```
   → All sensors initialized
   → Motors set to safe defaults
   → Wiper cycle starts immediately
   → Servo positioned at 90° (neutral)
   ```

2. **Continuous Monitoring Loop**
   ```
   → Read all sensor values
   → Process ultrasonic detection
   → Check metal detector
   → Monitor water levels
   → Assess turbidity
   → Detect rainfall
   → Calculate blockage risk
   ```

3. **Automated Responses**

   **Object Detected:**
   ```
   IF ultrasonic detects object within 20cm
   AND conveyor not already running
   THEN
     → Start conveyor motor (speed 145)
     → Run for 5 seconds
     → Automatic stop
     → Reset latch
   ```

   **Metal Detected:**
   ```
   IF metal detector triggered
   THEN
     → Activate 800ms latch
     → Rotate servo to 20°
     → Hold position during latch
     → Return to 90° when latch expires
   ```

   **Blockage Detected:**
   ```
   IF rain detected AND water present
   AND condition persists for 15 seconds
   THEN
     IF high water sensor triggered
       → SEVERE BLOCKAGE alert
     ELSE
       → PARTIAL BLOCKAGE alert
   ```

4. **Maintenance Operations**
   ```
   Wiper Cycle (continuous):
   → ON for 300ms (cleaning)
   → OFF for 2000ms (waiting)
   → Repeat indefinitely
   ```

### Blockage Detection Algorithm

```
Time Window: 15 seconds
Trigger Conditions: Rain + Water Low Level OK

Evaluation:
├─ Water High Level OK
│  └─ Result: SEVERE BLOCKAGE 🚨
│     (Water rising/overflow detected)
│
├─ Water High Level NOT OK
│  └─ Result: PARTIAL BLOCKAGE ⚠️
│     (Rain present but water not rising normally)
│
└─ Either condition false
   └─ Result: NORMAL FLOW ✅
      (System operating normally)
```

### Serial Output Format

Every 10 seconds, the system prints:
```
==============================
SENSOR READINGS + DECISIONS

Water Low  (A0): 645  => OK
Water High (A3): 423  => LOW
Turbidity  (A1): 789  => DIRTY
Rain (A2): 678  => WET
Metal DO (D36 raw): 1  => NO METAL
Ultrasonic (37/38): 15 cm  => OBJECT DETECTED
Conveyor Motor: RUNNING (3s left)
Wiper Motor: ON (wiping)
Wiper PWM Speed: 40
✅ WATER FLOW NORMAL
==============================
```

## 🎯 Threshold Tuning

### Water Level Sensors

**Calibration Process:**
```
1. Place sensor in completely dry environment
   → Note the reading (baseline)
2. Gradually submerge in water
   → Record readings at different depths
3. Identify reliable trigger points
4. Set thresholds with 10% safety margin
```

**Recommended Settings:**
- **Low sensor**: Set threshold where consistent water presence starts
- **High sensor**: Set threshold before overflow point

### Turbidity Sensor

**Calibration Process:**
```
1. Test with clean, clear water
   → Record baseline (should be low value)
2. Add dirt/sediment gradually
   → Note when water becomes visibly cloudy
3. Set threshold at noticeable cloudiness point
```

### Rain Sensor

**Calibration Process:**
```
1. Dry condition test
   → Note reading (high value typically)
2. Spray with water
   → Record wet reading (low value typically)
3. Set threshold between dry and wet states
```

### Ultrasonic Sensor

**Calibration Process:**
```
1. Test detection range with various objects
2. Adjust DETECT_CM based on drain width
3. Consider: Smaller value = more precise
4. Typical range: 10-30cm
```

### Tips for Tuning

- **Start conservative**: Use higher thresholds and reduce if needed
- **Environmental factors**: Temperature affects sensors
- **Regular recalibration**: Monthly checks recommended
- **Document settings**: Keep log of working thresholds
- **Test edge cases**: Extreme conditions (heavy rain, full blockage)

## 🐛 Troubleshooting

### Conveyor Motor Not Starting

**Symptoms**: Object detected but motor doesn't run

**Solutions**:
1. Check power supply to L298N driver
2. Verify pin connections (EN, IN1, IN2)
3. Test with higher CONV_SPEED value
4. Check motor connections and polarity
5. Ensure conveyor isn't already running

### Metal Detection Unreliable

**Symptoms**: False positives or missed detections

**Solutions**:
1. Verify INPUT_PULLUP is set correctly
2. Check metal detector module connections
3. Adjust METAL_LATCH_MS (increase for stability)
4. Ensure metal detector has proper power
5. Test with different metal objects
6. Check for electromagnetic interference

### Wiper Motor Issues

**Symptoms**: Wiper doesn't move or moves erratically

**Solutions**:
1. Check 12V power supply
2. Increase WIPER_SPEED if too slow
3. Verify L298N connections
4. Test motor separately
5. Check for mechanical obstructions

### Ultrasonic Sensor No Reading

**Symptoms**: "No reading" message in serial output

**Solutions**:
1. Check trigger and echo pin connections
2. Ensure sensor has 5V power
3. Verify no obstructions in sensor path
4. Test with known distance object
5. Check for loose wires
6. Replace sensor if defective

### Blockage False Alarms

**Symptoms**: Blockage alerts with no actual blockage

**Solutions**:
1. Increase BLOCKAGE_TIME (try 20-30 seconds)
2. Recalibrate water level sensors
3. Verify rain sensor isn't giving false positives
4. Check water sensor placement
5. Adjust threshold values

### Serial Monitor Shows Garbage

**Symptoms**: Unreadable characters in serial output

**Solutions**:
1. Ensure baud rate is set to 9600
2. Check USB cable connection
3. Restart Arduino IDE
4. Try different USB port

### System Resets Unexpectedly

**Symptoms**: Arduino reboots during operation

**Solutions**:
1. **Power issue** (most common):
   - Use external power supply for motors
   - Ensure sufficient current capacity
   - Add capacitors near motor drivers
2. Check for short circuits
3. Verify voltage regulator not overheating
4. Reduce simultaneous motor operations

## 📊 Performance Characteristics

- **Response Time**: < 100ms for all sensors
- **Monitoring Frequency**: 10-second intervals
- **Conveyor Cycle**: 5 seconds per activation
- **Wiper Cycle**: 2.3 seconds total (300ms ON + 2000ms OFF)
- **Metal Detection Stability**: 800ms latch window
- **Blockage Detection**: 15-second evaluation window
- **Power Consumption**: ~500mA (Arduino) + motor current

## 🔮 Future Enhancements

- [ ] IoT integration with cloud monitoring
- [ ] Mobile app for remote control
- [ ] Machine learning for predictive maintenance
- [ ] Additional water quality sensors (pH, temperature)
- [ ] Battery backup system
- [ ] Solar power integration
- [ ] Email/SMS alerts for critical conditions
- [ ] Data logging to SD card
- [ ] Web dashboard for analytics

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Contributors

- **Robiul Awal** - Initial development and documentation

## 📞 Support

For issues, questions, or contributions:
- **GitHub Issues**: [Report a bug](https://github.com/Robiulawal527/smartDrain/issues)
- **Email**: Contact through GitHub profile

## 🙏 Acknowledgments

- Arduino community for excellent documentation
- Open-source sensor libraries
- Contributors to drainage management research

---

**Last Updated**: January 2026
**Version**: 1.0.0
**Status**: Active Development