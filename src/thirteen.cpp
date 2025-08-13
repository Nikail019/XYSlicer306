#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h> // for TWCR/TWEN

// === PINOUTS ===
// Left motor + encoder
const int LM_DIR     = 4;
const int LM_PWM     = 5;
const int encoderA_L = 21;  // interrupt pin (SCL)
const int encoderB_L = 14;  // digital pin

// Right motor + encoder (swapped)
const int RM_DIR     = 7;   // direction pin
const int RM_PWM     = 6;   // PWM pin
const int encoderA_R = 20;  // interrupt pin (SDA)
const int encoderB_R = 15;  // digital pin

// Encoder counts
volatile int16_t encoderCountL = 0;
volatile int16_t encoderCountR = 0;

// === Disable I2C/TWI so pins 20/21 don't have pull-ups ===
static inline void disableTWI() {
  TWCR &= ~(1 << TWEN);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  digitalWrite(20, LOW);
  digitalWrite(21, LOW);
}

// === ENCODER ISRs ===
void EncoderISR_L() {
  if (digitalRead(encoderB_L)) encoderCountL++;
  else                         encoderCountL--;
}

void EncoderISR_R() {
  if (digitalRead(encoderB_R)) encoderCountR++;
  else                         encoderCountR--;
}

// === MOVE RIGHT BY ENCODER COUNT ===
// direction: HIGH = clockwise (+encoder counts), LOW = anticlockwise (-encoder counts)
// speed: 0..255
// counts: positive number of ticks to move from CURRENT position
void moveRightToEncoderCount(bool direction, int speed, int counts) {
  if (counts < 0) counts = -counts;           // ensure positive magnitude
  speed = constrain(speed, 0, 255);

  // Set direction and start
  digitalWrite(RM_DIR, direction ? HIGH : LOW);
  int16_t start = encoderCountR;

  if (direction == HIGH) {
    // target is higher than start
    int16_t target = start + counts;
    analogWrite(RM_PWM, speed);
    while (encoderCountR < target) {
      Serial.print("Encoder Count R: ");
    }
  } else {
    // target is lower than start
    int16_t target = start - counts;
    analogWrite(RM_PWM, speed);
    while (encoderCountR > target) {
      Serial.print("Encoder Count R: ");
    }
  }

  // stop motor
  analogWrite(RM_PWM, 0);
}

int main() {
  init();
  Serial.begin(115200);

  disableTWI();

  // Motor pins
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);

  // Encoder pins
  pinMode(encoderA_L, INPUT);
  pinMode(encoderB_L, INPUT);
  pinMode(encoderA_R, INPUT);
  pinMode(encoderB_R, INPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoderA_L), EncoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_R), EncoderISR_R, RISING);

  // Reset counts
  encoderCountL = 0;
  encoderCountR = 0;

  // === Example Moves ===
  while (1) {
    // Move right motor 200 counts in the + direction (clockwise per your note)
    moveRightToEncoderCount(HIGH, 150, 200);
    delay(1000); // wait for 1 second

    // Example: move back 200 counts in the - direction
    // moveRightToEncoderCount(LOW, 150, 200);
    // delay(1000);
  }

  return 0;
}