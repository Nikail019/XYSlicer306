#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>   // for TWCR/TWEN

// === PINOUTS ===
// Left motor + encoder
const int LM_DIR     = 4;
const int LM_PWM     = 5;
const int encoderA_L = 21;  // interrupt pin (SCL)
const int encoderB_L = 14;  // digital pin

// Right motor + encoder (pins swapped)
const int RM_DIR     = 7;   // direction pin (swapped)
const int RM_PWM     = 6;   // PWM pin (swapped)
const int encoderA_R = 20;  // interrupt pin (SDA)
const int encoderB_R = 15;  // digital pin

// Encoder counts
volatile int16_t encoderCountL = 0;
volatile int16_t encoderCountR = 0;

// === Disable I2C/TWI so 20/21 don't have pull-ups ===
static inline void disableTWI() {
  // Disable TWI peripheral (clear TWEN)
  TWCR &= ~(1 << TWEN);
  // Make sure internal pull-ups on 20/21 are off
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  digitalWrite(20, LOW);
  digitalWrite(21, LOW);
}

// === ENCODER ISRs ===
// Simple: edge on A, read B for direction
void EncoderISR_L() {
  if (digitalRead(encoderB_L)) encoderCountL++;
  else                         encoderCountL--;
}

void EncoderISR_R() {
  if (digitalRead(encoderB_R)) encoderCountR++;
  else                         encoderCountR--;
}

// === MOVE PER ENCODER COUNT FUNCTIONS (your originals) ===
void moveLeftToEncoderCount(bool direction, int speed, int targetDelta) {
  int16_t startCount = encoderCountL;

  digitalWrite(LM_DIR, direction ? HIGH : LOW);
  analogWrite(LM_PWM, speed);

  if (!direction) {
    while ((encoderCountL - startCount) < targetDelta) { /* wait */ }
  } else {
    while ((startCount - encoderCountL) < targetDelta) { /* wait */ }
  }

  analogWrite(LM_PWM, 0);
}

void moveRightToEncoderCount(bool direction, int speed, int targetDelta) {
  int16_t startCount = encoderCountR;

  digitalWrite(RM_DIR, direction ? HIGH : LOW);
  analogWrite(RM_PWM, speed);

  if (!direction) {
    while ((encoderCountR - startCount) < targetDelta) { /* wait */ }
  } else {
    while ((startCount - encoderCountR) < targetDelta) { /* wait */ }
  }

  analogWrite(RM_PWM, 0);
}

int main() {
  init();

  // Kill I2C/TWI so pins 20/21 are free of pull-ups
  disableTWI();

  // Motor pins
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);

  // Encoder pins (leave as INPUT; pull-ups off for 20/21 already)
  pinMode(encoderA_L, INPUT);
  pinMode(encoderB_L, INPUT);
  pinMode(encoderA_R, INPUT);
  pinMode(encoderB_R, INPUT);

  // Attach interrupts on A channels (RISING like your original)
  attachInterrupt(digitalPinToInterrupt(encoderA_L), EncoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_R), EncoderISR_R, RISING);

  // Reset counts
  encoderCountL = 0;
  encoderCountR = 0;

  // === Move both motors for 2000 encoder counts each (sequential) ===
  moveLeftToEncoderCount(HIGH, 150, 2000);
  moveRightToEncoderCount(HIGH, 150, 2000);

  return 0;
}