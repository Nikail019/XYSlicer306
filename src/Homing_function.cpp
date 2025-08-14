#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// =================== MOTOR PINS ===================
const int LM_DIR = 4;   // Left motor DIR
const int LM_PWM = 5;   // Left motor PWM  (Timer3 PWM channel)
const int RM_DIR = 7;   // Right motor DIR
const int RM_PWM = 6;   // Right motor PWM (Timer4 PWM channel)

// =================== LIMIT SWITCH PINS ===================
// TOP, RIGHT, LEFT, BOTTOM == D2, D3, D19, D18 (matches your original)
const int SW_TOP_PIN    = 2;
const int SW_RIGHT_PIN  = 3;
const int SW_LEFT_PIN   = 19;
const int SW_BOTTOM_PIN = 18;

// =================== HOMING FLAGS + DEBOUNCE ===================
volatile bool x_debounce_flag = false;  // for RIGHT/LEFT  (Timer1)
volatile bool y_debounce_flag = false;  // for TOP/BOTTOM (Timer5)  // <-- changed
volatile bool hit_top    = false;
volatile bool hit_right  = false;
volatile bool hit_left   = false;
volatile bool hit_bottom = false;

// (Only needed so we can zero them at the end, like your FSM code)
volatile int16_t encoderCountL = 0;
volatile int16_t encoderCountR = 0;

// =================== LIMIT SWITCH ISRs (same logic) ===================
void LimitTop() {
  if (y_debounce_flag == 0) {
    Serial.println("Top limit switch triggered.");
    TCNT5 = 0;                 // <-- changed (was TCNT3)
    hit_top = true;
    y_debounce_flag = true;
  }
}
void LimitRight() {
  if (x_debounce_flag == 0) {
    Serial.println("Right limit switch triggered.");
    TCNT1 = 0;
    hit_right = true;
    x_debounce_flag = true;
  }
}
void LimitLeft() {
  if (x_debounce_flag == 0) {
    Serial.println("Left limit switch triggered.");
    TCNT1 = 0;
    hit_left = true;
    x_debounce_flag = true;
  }
}
void LimitBottom() {
  if (y_debounce_flag == 0) {
    Serial.println("Bottom limit switch triggered.");
    TCNT5 = 0;                 // <-- changed (was TCNT3)
    hit_bottom = true;
    y_debounce_flag = true;
  }
}

// Debounce timer ISRs (clear the gates)
ISR(TIMER1_COMPA_vect) { x_debounce_flag = false; }  // RIGHT/LEFT debounce window done
ISR(TIMER5_COMPA_vect) { y_debounce_flag = false; }  // TOP/BOTTOM debounce window done  // <-- changed

// =================== INIT (switches + motors) ===================

// =================== HOMING FUNCTION (extracted) ===================
void simulateHoming() {
  Serial.println("Homing BRuv");

  // Move horizontal left till LEFT switch hit — both motors CCW
  digitalWrite(RM_DIR, LOW);
  digitalWrite(LM_DIR, LOW);

  Serial.println("Moving left to hit left limit switch...");
  analogWrite(RM_PWM, 200);  // a bit above 100 to overcome static friction
  analogWrite(LM_PWM, 200);  // works now because Timer3 is free for PWM

  Serial.println("Waiting for left limit switch to be hit...");
  while (!hit_left) { /* blocking wait */ }

  analogWrite(RM_PWM, 0);
  analogWrite(LM_PWM, 0);
  Serial.println("Left limit switch hit, stopping motors.");
  delay(1000);

  // Move down till BOTTOM switch hit (Left CW, Right CCW)
  digitalWrite(RM_DIR, HIGH);  // adjust if needed by your kinematics
  digitalWrite(LM_DIR, LOW);
  analogWrite(RM_PWM, 150);
  analogWrite(LM_PWM, 155);

  Serial.println("Waiting for bottom limit switch to be hit...");
  while (!hit_bottom) { /* blocking wait */ }

  analogWrite(RM_PWM, 0);
  analogWrite(LM_PWM, 0);
  Serial.println("Bottom limit switch hit, stopping motors.");

  // Reset encoder counts
  encoderCountL = 0;
  encoderCountR = 0;

  Serial.println("Homing complete.");
}

// =================== MAIN ===================
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

  // Limit switch interrupts (RISING, same as your original)
  attachInterrupt(digitalPinToInterrupt(SW_TOP_PIN),    LimitTop,    RISING);
  attachInterrupt(digitalPinToInterrupt(SW_RIGHT_PIN),  LimitRight,  RISING);
  attachInterrupt(digitalPinToInterrupt(SW_LEFT_PIN),   LimitLeft,   RISING);
  attachInterrupt(digitalPinToInterrupt(SW_BOTTOM_PIN), LimitBottom, RISING);

  // Debounce timers:
  // Timer1 -> X side (RIGHT/LEFT)  [unchanged]
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);               // CTC
  TCCR1B |= (1 << CS12) | (1 << CS10);  // prescaler 1024
  OCR1A = 3125;                         // ~200 ms
  TIMSK1 |= (1 << OCIE1A);              // enable compare A interrupt

  // Timer5 -> Y side (TOP/BOTTOM)  [moved off Timer3]  // <-- changed
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= (1 << WGM52);               // CTC (WGM52=1, WGM53:0=0)
  TCCR5B |= (1 << CS52) | (1 << CS50);  // prescaler 1024
  OCR5A = 3125;                         // ~200 ms
  TIMSK5 |= (1 << OCIE5A);              // enable compare A interrupt
  sei();

  homingInit();
  simulateHoming();

  while (1) { /* idle */ }
  return 0;
}