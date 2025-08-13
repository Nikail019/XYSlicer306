#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>   // TWCR/TWEN

// ====== TUNING ======
#define Kp 5.0
#define Ki 0.2
#define MAX_PWM 255
#define LOOP_MS 10
#define TOL 2       // stop when |error| <= TOL

// ====== PINOUTS ======
// Left motor + encoder
const int LM_DIR     = 4;
const int LM_PWM     = 5;
const int encoderA_L = 21;  // SCL (INT2)
const int encoderB_L = 14;

// Right motor + encoder (swapped)
const int RM_DIR     = 7;   // dir
const int RM_PWM     = 6;   // pwm
const int encoderA_R = 20;  // SDA (INT3)
const int encoderB_R = 15;

// ====== STATE ======
volatile int16_t encoderCountL = 0;
volatile int16_t encoderCountR = 0;

// ====== HW HELPERS ======
static inline void disableTWI() {
  TWCR &= ~(1 << TWEN);   // disable I2C/TWI so 20/21 don't force pull-ups
  pinMode(20, INPUT); pinMode(21, INPUT);
  digitalWrite(20, LOW);  digitalWrite(21, LOW);
}

// ====== ENCODER ISRs (simple: A rising edge, read B for dir) ======
void EncoderISR_L() { if (digitalRead(encoderB_L)) encoderCountL++; else encoderCountL--; }
void EncoderISR_R() { if (digitalRead(encoderB_R)) encoderCountR++; else encoderCountR--; }

// ====== MOTOR DRIVE ======
static inline void MoveLeft(bool dir, int pwm)  { digitalWrite(LM_DIR, dir?HIGH:LOW); analogWrite(LM_PWM, constrain(pwm,0,MAX_PWM)); }
static inline void MoveRight(bool dir, int pwm) { digitalWrite(RM_DIR, dir?HIGH:LOW); analogWrite(RM_PWM, constrain(pwm,0,MAX_PWM)); }

// ====== SIMPLE PI: single-motor to absolute target ======
void moveLeftPITo(int targetCount) {
  float I = 0;
  while (true) {
    int e = targetCount - encoderCountL;
    if (abs(e) <= TOL) { analogWrite(LM_PWM, 0); break; }
    I += e;
    float u = Kp*e + Ki*I;
    MoveLeft(u > 0, abs((int)u));
    delay(LOOP_MS);
  }
}

void moveRightPITo(int targetCount) {
  float I = 0;
  while (true) {
    int e = targetCount - encoderCountR;
    if (abs(e) <= TOL) { analogWrite(RM_PWM, 0); break; }
    I += e;
    float u = Kp*e + Ki*I;
    MoveRight(u > 0, abs((int)u));
    delay(LOOP_MS);
  }
}

// ====== SIMPLE PI: both motors simultaneously ======
void moveBothPITo(int leftTarget, int rightTarget) {
  float IL = 0, IR = 0;
  while (true) {
    int eL = leftTarget  - encoderCountL;
    int eR = rightTarget - encoderCountR;

    bool doneL = (abs(eL) <= TOL);
    bool doneR = (abs(eR) <= TOL);
    if (doneL && doneR) break;

    if (doneL) { analogWrite(LM_PWM, 0); }
    else {
      IL += eL;
      float uL = Kp*eL + Ki*IL;
      MoveLeft(uL > 0, abs((int)uL));
    }

    if (doneR) { analogWrite(RM_PWM, 0); }
    else {
      IR += eR;
      float uR = Kp*eR + Ki*IR;
      MoveRight(uR > 0, abs((int)uR));
    }

    delay(LOOP_MS);
  }
  analogWrite(LM_PWM, 0);
  analogWrite(RM_PWM, 0);
}

int main() {
  init();

  // free SDA/SCL pull-ups
  disableTWI();

  // pins
  pinMode(LM_DIR, OUTPUT); pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT); pinMode(RM_PWM, OUTPUT);
  pinMode(encoderA_L, INPUT_PULLUP); pinMode(encoderB_L, INPUT_PULLUP);
  pinMode(encoderA_R, INPUT_PULLUP); pinMode(encoderB_R, INPUT_PULLUP);

  // interrupts on A channels (RISING like your original)
  attachInterrupt(digitalPinToInterrupt(encoderA_L), EncoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_R), EncoderISR_R, RISING);

  // zero counts
  encoderCountL = 0; encoderCountR = 0;

  // ====== EXAMPLES ======
  // Single motor:
  // moveLeftPITo(2000);
  // moveRightPITo(2000);

  // Both at once to 2000 counts:
  moveBothPITo(2000, 2000);

  return 0;
}