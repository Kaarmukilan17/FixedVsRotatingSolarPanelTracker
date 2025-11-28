#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Servo.h>

Servo sg90;

// --- Pins ---
const int LDR1_PIN = A1;   // Left
const int LDR2_PIN = A0;   // Right
const int SERVO_PIN = 4;

// --- Servo parameters ---
int position = 90;
const int posMin = 0;
const int posMax = 180;

// --- Control parameters ---
int fixedDeadband = 30;
const float smoothAlpha = 0.15;
const int maxStep = 5;
const float scale = 60.0;

// --- Internal variables ---
int offset = 0;
float smoothDiff = 0.0;

// --- INA219 sensors ---
Adafruit_INA219 inaFixed(0x40);
Adafruit_INA219 inaRotating(0x41);
0
// --- Helper: median read ---
int readMedian(int pin, int samples = 5) {
  int vals[9];
  for (int i = 0; i < samples; i++) {
    vals[i] = analogRead(pin);
    delay(5);
  }
  for (int i = 1; i < samples; i++) {
    int key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) {
      vals[j + 1] = vals[j];
      j--;
    }
    vals[j + 1] = key;
  }
  return vals[samples / 2];
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  sg90.attach(SERVO_PIN);
  pinMode(LDR1_PIN, INPUT);
  pinMode(LDR2_PIN, INPUT);

  // --- Center the servo ---
  sg90.write(position);
  delay(1500);

  // --- Calibrate LDR offset ---
  Serial.println("Calibrating LDR offset...");
  long sumDiff = 0;
  const int samples = 40;
  for (int i = 0; i < samples; i++) {
    sumDiff += analogRead(LDR1_PIN) - analogRead(LDR2_PIN);
    delay(30);
  }
  offset = sumDiff / samples;
  Serial.print("Offset: "); Serial.println(offset);

  // --- Initialize INA219s ---
  if (!inaFixed.begin()) Serial.println("INA219 (fixed) not found!");
  if (!inaRotating.begin()) Serial.println("INA219 (rotating) not found!");

  // --- CSV header for Python ---
  Serial.println("LDR1,LDR2,smoothDiff,pos,V_fixed,I_fixed,P_fixed,V_rot,I_rot,P_rot");
}

void loop() {
  // --- Read sensors ---
  int left = readMedian(LDR1_PIN, 5);
  int right = readMedian(LDR2_PIN, 5);
  int rawDiff = (left - right) - offset;

  // --- Smooth diff ---
  smoothDiff = (smoothAlpha * rawDiff) + ((1.0 - smoothAlpha) * smoothDiff);
  float absSmooth = fabs(smoothDiff);

  // --- Servo control ---
  if (absSmooth > fixedDeadband) {
    int step = constrain((int)round(smoothDiff / scale), -maxStep, maxStep);
    position = constrain(position - step, posMin, posMax);
    sg90.write(position);
  }

  // --- INA219 readings ---
  float V_fixed = inaFixed.getBusVoltage_V();
  float I_fixed = inaFixed.getCurrent_mA();
  float P_fixed =abs(V_fixed*I_fixed) ;

  float V_rot = inaRotating.getBusVoltage_V();
  float I_rot = inaRotating.getCurrent_mA();
  float P_rot = abs(I_rot*V_rot);

  // --- CSV output for Python ---
  Serial.print(left); Serial.print(",");
  Serial.print(right); Serial.print(",");
  Serial.print(smoothDiff, 2); Serial.print(",");
  Serial.print(position); Serial.print(",");
  Serial.print(V_fixed, 3); Serial.print(",");
  Serial.print(I_fixed, 2); Serial.print(",");
  Serial.print(P_fixed, 2); Serial.print(",");
  Serial.print(V_rot, 3); Serial.print(",");
  Serial.print(I_rot, 2); Serial.print(",");
  Serial.println(P_rot, 2);

  delay(1000);  // sample every second
}