#include <Servo.h>

// ================= ANALOG SENSOR PINS =================
#define WATER_LOW_PIN   A0
#define TURB_PIN        A1
#define RAIN_PIN        A2
#define WATER_HIGH_PIN  A3

// ================= ULTRASONIC (MEGA) =================
#define ULTRA_TRIG      37
#define ULTRA_ECHO      38

// ================= METAL DETECTOR =================
#define METAL_PIN       36   // digital input (DO)

// ================= WIPER MOTOR PINS =================
#define WIPER_EN   5
#define WIPER_IN1  30
#define WIPER_IN2  31

// ================= CONVEYOR MOTOR PINS =================
#define CONV_EN    6
#define CONV_IN1   32
#define CONV_IN2   33

// ================= SERVO =================
#define SERVO_PIN  9
Servo metalServo;

// ================= THRESHOLDS (tune if needed) =================
#define WATER_LOW_TH       500
#define WATER_HIGH_TH      650
#define TURB_DIRTY_TH      600
#define RAIN_WET_TH        500   // (kept, only displayed)

// Ultrasonic detect distance (cm)
#define DETECT_CM          20

// Conveyor motor run time (ms)
#define MOTOR_RUN_MS       5000UL

// Conveyor motor speed (0-255)
#define CONV_SPEED         145

// ✅ Print every 10 seconds
#define PRINT_EVERY_MS     10000UL

// ================= WIPER SETTINGS =================
const unsigned long WIPER_OFF_TIME = 2000UL;  // you set this (kept same)
const unsigned long WIPER_ON_TIME  = 300UL;   // you set this (kept same)
const int WIPER_SPEED = 40;                  // you set this (kept same)

// ================= METAL DETECT IMPROVEMENTS =================
const unsigned long METAL_LATCH_MS = 800UL;   // keep "detected" true for 0.8s after a hit
bool metalLatched = false;
unsigned long metalLatchStartMs = 0;

// ================= ULTRASONIC MOTOR STATE =================
bool objectLatched = false;
bool conveyorRunning = false;
unsigned long conveyorStartMs = 0;

// ================= WIPER STATE =================
bool wiperRunning = false;
unsigned long wiperStateStartMs = 0;

// ================= PRINT TIMER =================
unsigned long lastPrintMs = 0;

// ================= WATER BLOCKAGE (ADDED ONLY) =================
// Idea: when rain is wet and "water is present" for a while but the high sensor doesn't confirm rising,
// treat it as partial blockage; if waterHigh becomes OK (overflow/rising) => severe blockage.
bool blockage = false;             // severe
bool partialBlockage = false;      // mild/partial
unsigned long waterStartTime = 0;  // when "water flow condition" started
const unsigned long BLOCKAGE_TIME = 15000UL; // 15s window (tune)

// ----------------- Motor Helpers -----------------
void conveyorStop() {
  analogWrite(CONV_EN, 0);
  digitalWrite(CONV_IN1, LOW);
  digitalWrite(CONV_IN2, LOW);
}

void conveyorForward() {
  digitalWrite(CONV_IN1, HIGH);
  digitalWrite(CONV_IN2, LOW);
  analogWrite(CONV_EN, CONV_SPEED);
}

void wiperStop() {
  analogWrite(WIPER_EN, 0);
  digitalWrite(WIPER_IN1, LOW);
  digitalWrite(WIPER_IN2, LOW);
}

void wiperForward() {
  digitalWrite(WIPER_IN1, HIGH);
  digitalWrite(WIPER_IN2, LOW);
  analogWrite(WIPER_EN, WIPER_SPEED); // slow speed applied
}

// ----------------- Ultrasonic Read -----------------
long readUltrasonicCM() {
  digitalWrite(ULTRA_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG, LOW);

  unsigned long duration = pulseIn(ULTRA_ECHO, HIGH, 30000UL);
  if (duration == 0) return -1;

  long cm = (long)(duration * 0.0343 / 2.0);
  return cm;
}

void setup() {
  Serial.begin(9600);

  // Ultrasonic
  pinMode(ULTRA_TRIG, OUTPUT);
  pinMode(ULTRA_ECHO, INPUT);
  digitalWrite(ULTRA_TRIG, LOW);

  // ✅ Metal: use pullup to avoid floating DO
  pinMode(METAL_PIN, INPUT_PULLUP);

  // Motors
  pinMode(WIPER_EN, OUTPUT);
  pinMode(WIPER_IN1, OUTPUT);
  pinMode(WIPER_IN2, OUTPUT);

  pinMode(CONV_EN, OUTPUT);
  pinMode(CONV_IN1, OUTPUT);
  pinMode(CONV_IN2, OUTPUT);

  // Safe defaults
  wiperStop();
  conveyorStop();

  // Start wiper cycle: first wipe happens immediately
  wiperRunning = true;
  wiperStateStartMs = millis();
  wiperForward();

  // Servo
  metalServo.attach(SERVO_PIN);
  metalServo.write(90);

  Serial.println("=== SYSTEM STARTED ===");
  Serial.println("Conveyor: ultrasonic detect => run 5s (speed 150) => stop");
  Serial.println("Wiper: ON for WIPER_ON_TIME, OFF for WIPER_OFF_TIME, repeats");
  Serial.println("Metal: DO is stabilized with INPUT_PULLUP + latch window");
}

void loop() {
  // --------- Time ----------
  unsigned long now = millis();

  // --------- Read sensors ----------
  int waterLow   = analogRead(WATER_LOW_PIN);
  int turb       = analogRead(TURB_PIN);
  int rain       = analogRead(RAIN_PIN);       // kept for display only
  int waterHigh  = analogRead(WATER_HIGH_PIN);

  // Read raw metal DO (with pullup)
  int metalRaw = digitalRead(METAL_PIN);
  bool metalHit = (metalRaw == LOW); // best guess with INPUT_PULLUP wiring

  long distCm = readUltrasonicCM();

  // --------- Conditions ----------
  bool waterLowOK  = (waterLow  > WATER_LOW_TH);
  bool waterHighOK = (waterHigh > WATER_HIGH_TH);
  bool dirtyWater  = (turb      > TURB_DIRTY_TH);

  bool rainWet = (rain > RAIN_WET_TH); // (only for blockage logic + display)

  bool detected = (distCm > 0 && distCm <= DETECT_CM);

  // --------- ULTRASONIC => CONVEYOR 5s cycle ----------
  if (detected && !objectLatched && !conveyorRunning) {
    objectLatched = true;
    conveyorRunning = true;
    conveyorStartMs = now;
    conveyorForward();
  }
  if (!detected) objectLatched = false;

  if (conveyorRunning && (now - conveyorStartMs >= MOTOR_RUN_MS)) {
    conveyorStop();
    conveyorRunning = false;
  }

  // --------- WIPER cycle (same logic you already used) ----------
  if (wiperRunning) {
    if (now - wiperStateStartMs >= WIPER_ON_TIME) {
      wiperStop();
      wiperRunning = false;
      wiperStateStartMs = now;
    }
  } else {
    if (now - wiperStateStartMs >= WIPER_OFF_TIME) {
      wiperForward();
      wiperRunning = true;
      wiperStateStartMs = now;
    }
  }

  // --------- METAL latch (makes it feel more sensitive) ----------
  if (metalHit) {
    metalLatched = true;
    metalLatchStartMs = now;
  }
  if (metalLatched && (now - metalLatchStartMs >= METAL_LATCH_MS)) {
    metalLatched = false;
  }

  // --------- Metal Servo ----------
  if (metalLatched) metalServo.write(20);  // detected position
  else metalServo.write(90);               // neutral position

  // ================= WATER BLOCKAGE LOGIC (ADDED ONLY) =================
  // Start timing when rain is wet and waterLow indicates water presence.
  // After BLOCKAGE_TIME:
  //  - If waterHighOK => severe blockage (water rising / overflow).
  //  - Else => partial blockage (rain + water present but not rising normally).
  if (rainWet && waterLowOK) {
    if (waterStartTime == 0) {
      waterStartTime = now;
    }

    if ((now - waterStartTime) >= BLOCKAGE_TIME) {
      if (waterHighOK) {
        blockage = true;
        partialBlockage = false;
      } else {
        partialBlockage = true;
        blockage = false;
      }
    }
  } else {
    // reset when not in water-flow condition
    waterStartTime = 0;
    blockage = false;
    partialBlockage = false;
  }

  // --------- Serial Output every 10 seconds ----------
  if (now - lastPrintMs >= PRINT_EVERY_MS) {
    lastPrintMs = now;

    Serial.println("\n==============================");
    Serial.println("SENSOR READINGS + DECISIONS");

    Serial.print("Water Low  (A0): "); Serial.print(waterLow);
    Serial.print("  => "); Serial.println(waterLowOK ? "OK" : "LOW");

    Serial.print("Water High (A3): "); Serial.print(waterHigh);
    Serial.print("  => "); Serial.println(waterHighOK ? "OK" : "LOW");

    Serial.print("Turbidity  (A1): "); Serial.print(turb);
    Serial.print("  => "); Serial.println(dirtyWater ? "DIRTY" : "CLEAN");

    Serial.print("Rain (A2): "); Serial.print(rain);
    Serial.print("  => "); Serial.println(rainWet ? "WET" : "DRY");

    Serial.print("Metal DO (D36 raw): "); Serial.print(metalRaw);
    Serial.print("  => "); Serial.println(metalLatched ? "METAL DETECTED (LATCHED)" : "NO METAL");

    Serial.print("Ultrasonic (37/38): ");
    if (distCm < 0) Serial.println("No reading");
    else {
      Serial.print(distCm); Serial.print(" cm  => ");
      Serial.println(detected ? "OBJECT DETECTED" : "CLEAR");
    }

    Serial.print("Conveyor Motor: ");
    if (conveyorRunning) {
      unsigned long elapsed = now - conveyorStartMs;
      unsigned long left = (elapsed >= MOTOR_RUN_MS) ? 0 : (MOTOR_RUN_MS - elapsed);
      Serial.print("RUNNING ("); Serial.print(left / 1000); Serial.println("s left)");
    } else {
      Serial.println("STOPPED");
    }

    Serial.print("Wiper Motor: ");
    Serial.println(wiperRunning ? "ON (wiping)" : "OFF (waiting)");

    Serial.print("Wiper PWM Speed: "); Serial.println(WIPER_SPEED);

    // ✅ Blockage decision print (added only)
    if (blockage) Serial.println("🚨 SEVERE WATER BLOCKAGE DETECTED");
    else if (partialBlockage) Serial.println("⚠️ PARTIAL WATER BLOCKAGE");
    else Serial.println("✅ WATER FLOW NORMAL");

    Serial.println("==============================");
  }
}
