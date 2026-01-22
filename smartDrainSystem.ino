#include <Servo.h>

// ================= SENSOR PINS =================
#define WATER_LOW_PIN   A0
#define TURB_PIN        A1
#define RAIN_PIN        A2
#define WATER_HIGH_PIN  A3

// Ultrasonic #1 (main / conveyor trigger)
#define ULTRA1_TRIG     34
#define ULTRA1_ECHO     35

// Ultrasonic #2 (Bin-1 level)
#define ULTRA2_TRIG     37
#define ULTRA2_ECHO     38

// Ultrasonic #3 (Bin-2 level)
#define ULTRA3_TRIG     39
#define ULTRA3_ECHO     40

#define METAL_PIN       36   // keep unchanged (occupied range allowed)

// ================= WIPER MOTOR (OUT1/OUT2) =================
#define WIPER_EN   5     // ENA (PWM)
#define WIPER_IN1  30
#define WIPER_IN2  31

// ================= CONVEYOR MOTOR (OUT3/OUT4) =================
#define CONV_EN    6     // ENB (PWM)
#define CONV_IN1   32
#define CONV_IN2   33

// ================= SERVO =================
#define SERVO_PIN  9
Servo metalServo;

// ================= THRESHOLDS =================
#define WATER_LOW_TH        500
#define WATER_HIGH_TH       650
#define TURB_DIRTY_TH       500
#define RAIN_HEAVY_TH       500

#define ULTRA1_DISTANCE_TH  15   // cm: object near main ultrasonic -> start conveyor

// Bin thresholds (tune for your bin height)
// Example: if bin is "full", distance becomes small (sensor near top).
#define BIN_FULL_CM         8    // cm: <= this means bin is full

// ================= WIPER SETTINGS =================
const unsigned long WIPER_OFF_TIME = 20000; // ms stop time
const unsigned long WIPER_ON_TIME  = 100;   // ms on pulse
const int WIPER_SPEED = 100;                // 0-255 PWM

// ================= CONVEYOR SETTINGS =================
const unsigned long CONVEYOR_RUN_TIME = 60000; // ms
const int CONV_SPEED = 200;                    // 0-255 PWM

// ================= VARIABLES =================
bool wiperRunning = false;
unsigned long wiperTimer = 0;

bool conveyorRunning = false;
unsigned long conveyorTimer = 0;

// Serial printing
unsigned long printTimer = 0;
const unsigned long PRINT_INTERVAL = 300;

// Metal trigger lockout
unsigned long lastMetalTrigger = 0;
const unsigned long METAL_LOCKOUT = 3000;

// ---------- Ultrasonic helper ----------
long readUltrasonicCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Timeout prevents blocking forever
  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return -1;              // no echo / out of range / wiring issue
  return (long)(duration * 0.034 / 2.0);     // cm
}

void setup() {
  Serial.begin(9600);
  Serial.println("Smart Drain + Wiper + Conveyor + 3xUltrasonic + Metal Servo Started");

  // Wiper motor pins
  pinMode(WIPER_EN, OUTPUT);
  pinMode(WIPER_IN1, OUTPUT);
  pinMode(WIPER_IN2, OUTPUT);
  digitalWrite(WIPER_IN1, LOW);
  digitalWrite(WIPER_IN2, LOW);
  analogWrite(WIPER_EN, 0);

  // Conveyor motor pins
  pinMode(CONV_EN, OUTPUT);
  pinMode(CONV_IN1, OUTPUT);
  pinMode(CONV_IN2, OUTPUT);
  digitalWrite(CONV_IN1, LOW);
  digitalWrite(CONV_IN2, LOW);
  analogWrite(CONV_EN, 0);

  // Ultrasonic pins
  pinMode(ULTRA1_TRIG, OUTPUT); pinMode(ULTRA1_ECHO, INPUT); digitalWrite(ULTRA1_TRIG, LOW);
  pinMode(ULTRA2_TRIG, OUTPUT); pinMode(ULTRA2_ECHO, INPUT); digitalWrite(ULTRA2_TRIG, LOW);
  pinMode(ULTRA3_TRIG, OUTPUT); pinMode(ULTRA3_ECHO, INPUT); digitalWrite(ULTRA3_TRIG, LOW);

  // Metal detector input (stable)
  pinMode(METAL_PIN, INPUT_PULLUP);

  // Servo
  metalServo.attach(SERVO_PIN);
  metalServo.write(0);

  wiperTimer = millis();
  printTimer = millis();
}

void loop() {
  unsigned long now = millis();

  // ---------- Read analog sensors ----------
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

  // ---------- Read ultrasonics ONE BY ONE (avoid interference) ----------
  long distMain = readUltrasonicCM(ULTRA1_TRIG, ULTRA1_ECHO);
  delay(30);
  long distBin1 = readUltrasonicCM(ULTRA2_TRIG, ULTRA2_ECHO);
  delay(30);
  long distBin2 = readUltrasonicCM(ULTRA3_TRIG, ULTRA3_ECHO);

  bool bin1Full = (distBin1 > 0 && distBin1 <= BIN_FULL_CM);
  bool bin2Full = (distBin2 > 0 && distBin2 <= BIN_FULL_CM);

  // ---------- WIPER timing ----------
  if (!wiperRunning && now - wiperTimer >= WIPER_OFF_TIME) {
    wiperRunning = true;
    wiperTimer = now;

    digitalWrite(WIPER_IN1, HIGH);
    digitalWrite(WIPER_IN2, LOW);
    analogWrite(WIPER_EN, WIPER_SPEED);
  }

  if (wiperRunning && now - wiperTimer >= WIPER_ON_TIME) {
    wiperRunning = false;
    wiperTimer = now;

    digitalWrite(WIPER_IN1, LOW);
    digitalWrite(WIPER_IN2, LOW);
    analogWrite(WIPER_EN, 0);
  }

  // ---------- CONVEYOR ----------
  // Optional safety: don't start if both bins are full
  bool binsAvailable = !(bin1Full && bin2Full);

  if (!conveyorRunning && binsAvailable && distMain > 0 && distMain <= ULTRA1_DISTANCE_TH) {
    conveyorRunning = true;
    conveyorTimer = now;

    digitalWrite(CONV_IN1, HIGH);
    digitalWrite(CONV_IN2, LOW);
    analogWrite(CONV_EN, CONV_SPEED);
  }

  if (conveyorRunning && now - conveyorTimer >= CONVEYOR_RUN_TIME) {
    conveyorRunning = false;

    digitalWrite(CONV_IN1, LOW);
    digitalWrite(CONV_IN2, LOW);
    analogWrite(CONV_EN, 0);
  }

  // ---------- METAL DETECTION & SERVO (ONCE PER DETECTION) ----------
  static bool lastMetalDetected = false;

  // With INPUT_PULLUP: many sensors read LOW when active.
  // If your module reads HIGH when active, change == LOW to == HIGH.
  bool metalDetected = (digitalRead(METAL_PIN) == LOW);

  if (metalDetected && !lastMetalDetected && (now - lastMetalTrigger > METAL_LOCKOUT)) {
    lastMetalTrigger = now;
    Serial.println("Metal Detected -> Servo once");

    metalServo.write(35);
    delay(400);
    metalServo.write(0);
  }
  lastMetalDetected = metalDetected;

  // ---------- PRINT ALL READINGS ----------
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

    Serial.print(" | Wiper: "); Serial.print(wiperRunning);
    Serial.print(" | Conveyor: "); Serial.print(conveyorRunning);

    Serial.print(" | Metal: "); Serial.print((int)metalDetected);
    Serial.println();
  }
}
