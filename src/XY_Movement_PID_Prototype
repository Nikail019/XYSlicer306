#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>   // for M_PI

// ==== Motor pins ====
const int LM_DIR = 4;   // Left motor DIR
const int LM_PWM = 5;   // Left motor PWM
const int RM_DIR = 7;   // Right motor DIR
const int RM_PWM = 6;   // Right motor PWM

// ==== Encoder pins (from your working ISR sketch) ====
const int encoderA_L = 21;  // interrupt pin (Left A)
const int encoderB_L = 14;  // digital pin  (Left B)
const int encoderA_R = 20;  // interrupt pin (Right A)
const int encoderB_R = 15;  // digital pin  (Right B)

// ==== Encoder counts (updated by ISRs) ====
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

// ==== Quadrature ISRs (A on RISING, read B to decide direction) ====
// Assumes: CW rotation should INCREASE counts.
// If you see the sign reversed for a motor, swap the ++/-- in that ISR.
void EncoderISR_L() {
  if (digitalRead(encoderB_L)) encoderCountL--;  // A rising, B=1 -> one direction (CW)
  else                         encoderCountL++;  // A rising, B=0 -> opposite (CCW)
}
void EncoderISR_R() {
  if (digitalRead(encoderB_R)) encoderCountR--;  // Same convention for right
  else                         encoderCountR++;
}

// ==== Your distance-to-encoder conversion (SIGNED) ====
// NB: X+Y or X-Y can be negative for CoreXY legs, so this MUST be signed.
long DistToEncoderCounts(float distance) {
  // this function converts a distance in mm to encoder counts
  float shaft_diameter = 12;               // shaft diameter in mm
  int   cpr_before_gear_box = 12;          // counts per revolution
  int   gear_ratio = 172;
  int   cpr_after_gear_box = cpr_before_gear_box * gear_ratio;
  float counts_to_rad_ratio = 2 * (float)M_PI / cpr_after_gear_box;
  float mm_per_count = 0.5f * counts_to_rad_ratio * shaft_diameter;   // = (pi*D)/cpr_after_gear_box
  float counts_f = distance / mm_per_count;                           // signed
  return (long)lroundf(counts_f);
}

// ==== Direction polarity from your observation: +X is HIGH/HIGH ====
constexpr uint8_t POS_DIR_L = HIGH;  // Left motor (A): positive counts => HIGH
constexpr uint8_t POS_DIR_R = HIGH;  // Right motor (B): positive counts => HIGH

static inline int sgn(long v) { return (v >= 0) ? +1 : -1; }

// ==== Move function (unchanged except your debug prints) ====
// Drives both motors simultaneously with a PWM ratio that matches distance-to-go, so both finish together.
void moveToXY_simple(float Xmm, float Ymm, uint8_t maxPWM = 200, uint8_t MIN_PWM = 70) {
  long tgtA = DistToEncoderCounts(Xmm + Ymm); // Left motor target (CoreXY: A = X + Y)
  long tgtB = DistToEncoderCounts(Xmm - Ymm); // Right motor target (CoreXY: B = X - Y)

  long curA = encoderCountL;
  long curB = encoderCountR;

  long dA = tgtA - curA;
  long dB = tgtB - curB;
  long absA = labs(dA);
  long absB = labs(dB);

  if (absA == 0 && absB == 0) return;

  // Directions
  digitalWrite(LM_DIR, (sgn(dA) > 0) ? POS_DIR_L : (POS_DIR_L == HIGH ? LOW : HIGH));
  digitalWrite(RM_DIR, (sgn(dB) > 0) ? POS_DIR_R : (POS_DIR_R == HIGH ? LOW : HIGH));

  Serial.print("Left motor direction: ");
  Serial.println((sgn(dA) > 0) ? "FORWARD" : "REVERSE");
  Serial.print("Right motor direction: ");  
  Serial.println((sgn(dB) > 0) ? "FORWARD" : "REVERSE");

  // PWM ratio so both arrive together
  uint8_t pwmA = 0, pwmB = 0; 
  if (absA == 0) {
    pwmA = 0;           pwmB = maxPWM;
  } else if (absB == 0) {
    pwmA = maxPWM;      pwmB = 0;
  } else if (absA >= absB) {
    pwmA = maxPWM;
    pwmB = (uint8_t)max((int)MIN_PWM, (int)((long)maxPWM * absB / absA));
  } else {
    pwmB = maxPWM;
    pwmA = (uint8_t)max((int)MIN_PWM, (int)((long)maxPWM * absA / absB));
  }

  // Start both motors
  analogWrite(LM_PWM, pwmA);
  analogWrite(RM_PWM, pwmB);

  // Block until both targets reached (stop each side independently when done)
  while (true) {
    // Debug print
    Serial.print("L: ");
    Serial.print(encoderCountL);
    Serial.print("  R: ");
    Serial.print(encoderCountR);
    Serial.print("  tgtA: ");
    Serial.print(tgtA);
    Serial.print("  tgtB: ");
    Serial.println(tgtB);

    bool Adone = ( (dA >= 0) ? (encoderCountL >= tgtA) : (encoderCountL <= tgtA) );
    bool Bdone = ( (dB >= 0) ? (encoderCountR >= tgtB) : (encoderCountR <= tgtB) );

    if (Adone && Bdone) break;
    if (Adone) analogWrite(LM_PWM, 0);
    if (Bdone) analogWrite(RM_PWM, 0);

    delay(50); // keeps serial readable
  }

  analogWrite(LM_PWM, 0);
  analogWrite(RM_PWM, 0);
}

// ==== MAIN ====
int main() {
  init();
  Serial.begin(9600);

  // Motor pins
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);
  analogWrite(LM_PWM, 0);
  analogWrite(RM_PWM, 0);

  // Encoder pins
  pinMode(encoderA_L, INPUT);
  pinMode(encoderB_L, INPUT);
  pinMode(encoderA_R, INPUT);
  pinMode(encoderB_R, INPUT);

  // Attach encoder interrupts on A channels (RISING)
  attachInterrupt(digitalPinToInterrupt(encoderA_L), EncoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_R), EncoderISR_R, RISING);

  // Zero encoders at start (you can also zero after homing)
  encoderCountL = 0;
  encoderCountR = 0;

  delay(500); // short pause before move

  // Example move
  Serial.println("Moving to (100, 10) mm");
  moveToXY_simple(100.0f, 10.0f, 200);


  delay(2000);
  moveToXY_simple(50.0f, 10.0f, 200);

  while (1) {
    // Idle
  }
  return 0;
}