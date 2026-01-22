#include <Servo.h>

// ================= ANALOG SENSOR PINS =================
#define WATER_LOW_PIN   A0
#define TURB_PIN        A1
#define RAIN_PIN        A2
#define WATER_HIGH_PIN  A3

// ================= ULTRASONIC SENSORS =================
// Main (conveyor trigger)
#define ULTRA1_TRIG     34
#define ULTRA1_ECHO     35
// Bin-1
#define ULTRA2_TRIG     37
#define ULTRA2_ECHO     38
// Bin-2
#define ULTRA3_TRIG     39
#define ULTRA3_ECHO     40

// ================= METAL + SERVO =================
#define METAL_PIN       36     // unchanged
#define SERVO_PIN       9
Servo metalServo;

// ================= WIPER MOTOR (OUT1/OUT2) =================
#define WIPER_EN   5     // ENA (PWM)
#define WIPER_IN1  30
#define WIPER_IN2  31

// ================= CONVEYOR MOTOR (OUT3/OUT4) =================
#define CONV_EN    6     // ENB (PWM)
#define CONV_IN1   32    // IN3
#define CONV_IN2   33    // IN4

// ================= THRESHOLDS =================
#define WATER_LOW_TH        500
#define WATER_HIGH_TH       650
#define TURB_DIRTY_TH       500
#define RAIN_HEAVY_TH       500

// Main ultrasonic: object detect distance
#define ULTRA1_DISTANCE_TH  15   // cm (tune)

// Bin thresholds (optional)
#define BIN_FULL_CM         8    // cm (tune)

// ================= WIPER SETTINGS =================
// Goal: one 360° rotation every 5 seconds.
// NOTE: Without an encoder/switch, “exactly 360°” is done by timing.
// Tune WIPER_ONE_ROT_MS until it makes ONE full rotation.
const unsigned long WIPER_INTERVAL_MS = 5000;   // every 5 seconds
const unsigned long WIPER_ONE_ROT_MS  = 325;    // <-- TUNE THIS (ms) for exactly 360°
const int WIPER_SPEED = 90;                    // 0-255 PWM (raise/lower if needed)

// ================= CONVEYOR SETTINGS =================
const int CONV_SPEED = 110;                     // slow speed (0-255) <-- adjust
const unsigned long CONV_MIN_RUN_MS = 10000;    // run at least 10 seconds once started
const unsigned long CONV_NO_DETECT_STOP_MS = 600; // after min run, stop if no detect for this long

// ================= METAL SERVO SETTINGS =================
const int SERVO_THROW_DEG = 35;
const unsigned long SERVO_HOLD_MS = 450;        // move/hold time
const unsigned long METAL_LOCKOUT = 3000;       // ignore repeated triggers

// ================= PRINT SETTINGS =================
const unsigned long PRINT_INTERVAL = 300;

// ================= STATE VARIABLES =================
bool wiperRunning = false;
unsigned long wiperNextStart = 0;
unsigned long wiperStopAt = 0;

bool conveyorRunning = false;
unsigned long conveyorStartAt = 0;
unsigned long lastConveyorDetectAt = 0;

// servo state (non-blocking)
bool servoMoving = false;
unsigned long servoMoveAt = 0;
unsigned long lastMetalTrigger = 0;
bool lastMetalDetected = false;

unsigned long printTimer = 0;

// ---------- Ultrasonic helper (non-blocking except pulseIn) ----------
long readUltrasonicCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 25000us timeout ≈ 4.3m
  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return -1;
  return (long)(duration * 0.034 / 2.0);
}

// Read ultrasonic with a tiny gap between sensors (avoids cross-talk)
long readUltrasonicSafe(uint8_t trigPin, uint8_t echoPin) {
  long d = readUltrasonicCM(trigPin, echoPin);
  // Small gap for echo to die out (no delay()—use microseconds)
  delayMicroseconds(3000); // 3ms
  return d;
}

// Simple detection: valid distance within threshold
bool isObjectDetected(long dist, int thresholdCm) {
  return (dist > 0 && dist <= thresholdCm);
}

void setup() {
  Serial.begin(9600);
  Serial.println("FINAL: Wiper + Conveyor + 3x Ultrasonic + Metal Servo");

  // Motor pins
  pinMode(WIPER_EN, OUTPUT);
  pinMode(WIPER_IN1, OUTPUT);
  pinMode(WIPER_IN2, OUTPUT);

  pinMode(CONV_EN, OUTPUT);
  pinMode(CONV_IN1, OUTPUT);
  pinMode(CONV_IN2, OUTPUT);

  digitalWrite(WIPER_IN1, LOW);
  digitalWrite(WIPER_IN2, LOW);
  analogWrite(WIPER_EN, 0);

  digitalWrite(CONV_IN1, LOW);
  digitalWrite(CONV_IN2, LOW);
  analogWrite(CONV_EN, 0);

  // Ultrasonic pins
  pinMode(ULTRA1_TRIG, OUTPUT); pinMode(ULTRA1_ECHO, INPUT); digitalWrite(ULTRA1_TRIG, LOW);
  pinMode(ULTRA2_TRIG, OUTPUT); pinMode(ULTRA2_ECHO, INPUT); digitalWrite(ULTRA2_TRIG, LOW);
  pinMode(ULTRA3_TRIG, OUTPUT); pinMode(ULTRA3_ECHO, INPUT); digitalWrite(ULTRA3_TRIG, LOW);

  // Metal detector input stable
  pinMode(METAL_PIN, INPUT_PULLUP);

  // Servo
  metalServo.attach(SERVO_PIN);
  metalServo.write(0);

  unsigned long now = millis();
  wiperNextStart = now + WIPER_INTERVAL_MS; // first wipe after 5 sec
  printTimer = now;
}

void loop() {
  unsigned long now = millis();

  // ================= READ ANALOG SENSORS =================
  int waterLow  = analogRead(WATER_LOW_PIN);
  int waterHigh = analogRead(WATER_HIGH_PIN);
  int turbValue = analogRead(TURB_PIN);
  int rainValue = analogRead(RAIN_PIN);

  bool waterRising   = (waterLow > WATER_LOW_TH);
  bool waterCritical = (waterHigh > WATER_HIGH_TH);
  bool waterDirty    = (turbValue < TURB_DIRTY_TH);
  bool heavyRain     = (rainValue < RAIN_HEAVY_TH);

  bool blockageDetected = (waterRising && waterDirty);
  bool overflowRisk     = (waterCritical && heavyRain);

  // ================= READ ULTRASONICS (3 units) =================
  // Read one-by-one to reduce interference
  long distMain = readUltrasonicSafe(ULTRA1_TRIG, ULTRA1_ECHO);
  long distBin1 = readUltrasonicSafe(ULTRA2_TRIG, ULTRA2_ECHO);
  long distBin2 = readUltrasonicSafe(ULTRA3_TRIG, ULTRA3_ECHO);

  bool mainDetected = isObjectDetected(distMain, ULTRA1_DISTANCE_TH);
  if (mainDetected) lastConveyorDetectAt = now;

  bool bin1Full = (distBin1 > 0 && distBin1 <= BIN_FULL_CM);
  bool bin2Full = (distBin2 > 0 && distBin2 <= BIN_FULL_CM);

  // ================= WIPER: one timed rotation every 5 sec =================
  // Start
  if (!wiperRunning && now >= wiperNextStart) {
    wiperRunning = true;
    wiperStopAt = now + WIPER_ONE_ROT_MS;

    digitalWrite(WIPER_IN1, HIGH);
    digitalWrite(WIPER_IN2, LOW);
    analogWrite(WIPER_EN, WIPER_SPEED);
  }

  // Stop
  if (wiperRunning && now >= wiperStopAt) {
    wiperRunning = false;

    digitalWrite(WIPER_IN1, LOW);
    digitalWrite(WIPER_IN2, LOW);
    analogWrite(WIPER_EN, 0);

    // schedule next wipe exactly 5 seconds after this stop
    wiperNextStart = now + WIPER_INTERVAL_MS;
  }

  // ================= CONVEYOR: slow speed + run logic =================
  // Start condition: object detected
  if (!conveyorRunning && mainDetected) {
    conveyorRunning = true;
    conveyorStartAt = now;
    // initialize detect time so it won't stop early
    lastConveyorDetectAt = now;

    digitalWrite(CONV_IN1, HIGH);
    digitalWrite(CONV_IN2, LOW);
    analogWrite(CONV_EN, CONV_SPEED);
  }

  // Stop condition:
  // - must have run at least 10 seconds, AND
  // - no object detection recently (or ultrasonic returns -1 / out of range)
  if (conveyorRunning) {
    bool minRunDone = (now - conveyorStartAt >= CONV_MIN_RUN_MS);
    bool noDetectRecently = (now - lastConveyorDetectAt >= CONV_NO_DETECT_STOP_MS);

    if (minRunDone && noDetectRecently) {
      conveyorRunning = false;
      digitalWrite(CONV_IN1, LOW);
      digitalWrite(CONV_IN2, LOW);
      analogWrite(CONV_EN, 0);
    }
  }

  // ================= METAL + SERVO: rotate once per detection =================
  // If your sensor triggers HIGH instead of LOW, change == LOW to == HIGH.
  bool metalDetected = (digitalRead(METAL_PIN) == LOW);

  // Trigger only on edge + lockout
  if (metalDetected && !lastMetalDetected && (now - lastMetalTrigger > METAL_LOCKOUT)) {
    lastMetalTrigger = now;
    servoMoving = true;
    servoMoveAt = now;

    metalServo.write(SERVO_THROW_DEG);
  }
  lastMetalDetected = metalDetected;

  // Return servo after hold time (non-blocking)
  if (servoMoving && (now - servoMoveAt >= SERVO_HOLD_MS)) {
    metalServo.write(0);
    servoMoving = false;
  }

  // ================= SERIAL PRINT =================
  if (now - printTimer >= PRINT_INTERVAL) {
    printTimer = now;

    Serial.print("WLlow: "); Serial.print(waterLow);
    Serial.print(" | WLhigh: "); Serial.print(waterHigh);
    Serial.print(" | Turb: "); Serial.print(turbValue);
    Serial.print(" | Rain: "); Serial.print(rainValue);

    Serial.print(" | Blockage: "); Serial.print(blockageDetected);
    Serial.print(" | Overflow: "); Serial.print(overflowRisk);

    Serial.print(" | Main(cm): "); Serial.print(distMain);
    Serial.print(" | Bin1(cm): "); Serial.print(distBin1);
    Serial.print(" | Bin2(cm): "); Serial.print(distBin2);
    Serial.print(" | Bin1Full: "); Serial.print(bin1Full);
    Serial.print(" | Bin2Full: "); Serial.print(bin2Full);

    Serial.print(" | WiperRun: "); Serial.print(wiperRunning);
    Serial.print(" | ConvRun: "); Serial.print(conveyorRunning);
    Serial.print(" | MainDet: "); Serial.print(mainDetected);

    Serial.print(" | MetalDet: "); Serial.print((int)metalDetected);
    Serial.println();
  }
}
