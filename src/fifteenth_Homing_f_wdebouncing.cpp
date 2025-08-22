#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// =================== MOTOR PINS ===================
const int LM_DIR = 4;   // Left motor DIR
const int LM_PWM = 5;   // Left motor PWM
const int RM_DIR = 7;   // Right motor DIR
const int RM_PWM = 6;   // Right motor PWM

// =================== LIMIT SWITCH PINS ===================
const int SW_TOP_PIN    = 2;
const int SW_RIGHT_PIN  = 3;
const int SW_LEFT_PIN   = 19;
const int SW_BOTTOM_PIN = 18;

// =================== FLAGS ===================
volatile bool hit_top    = false;
volatile bool hit_right  = false;
volatile bool hit_left   = false;
volatile bool hit_bottom = false;

// Debounce gates (X=LEFT/RIGHT on T1, Y=TOP/BOTTOM on T5)
volatile bool x_debounce_flag = false;
volatile bool y_debounce_flag = false;

// =================== ENCODER COUNTS (optional) ===================
volatile int16_t encoderCountL = 0;
volatile int16_t encoderCountR = 0;

// =================== LIMIT SWITCH ISRs (debounced) ===================
void LimitTop() {
  if (!y_debounce_flag) {
    // read current level so the flag reflects actual pressed/released state
    hit_top = digitalRead(SW_TOP_PIN);
    TCNT5 = 0;
    y_debounce_flag = true;
    // Serial.println("Top ISR");
  }
}
void LimitRight() {
  if (!x_debounce_flag) {
    hit_right = digitalRead(SW_RIGHT_PIN);
    TCNT1 = 0;
    x_debounce_flag = true;
    // Serial.println("Right ISR");
  }
}
void LimitLeft() {
  if (!x_debounce_flag) {
    hit_left = digitalRead(SW_LEFT_PIN);
    TCNT1 = 0;
    x_debounce_flag = true;
    // Serial.println("Left ISR");
  }
}
void LimitBottom() {
  if (!y_debounce_flag) {
    hit_bottom = digitalRead(SW_BOTTOM_PIN);
    TCNT5 = 0;
    y_debounce_flag = true;
    // Serial.println("Bottom ISR");
  }
}

// Debounce timer ISRs (re-open the gates after ~200 ms)
ISR(TIMER1_COMPA_vect) { x_debounce_flag = false; }
ISR(TIMER5_COMPA_vect) { y_debounce_flag = false; }

// =================== HOMING ===================
void simulateHoming() {
  Serial.println("Starting homing sequence...");

  // Move left until LEFT limit hit
  digitalWrite(RM_DIR, LOW);
  digitalWrite(LM_DIR, LOW);
  analogWrite(RM_PWM, 150);
  analogWrite(LM_PWM, 150);
  while (!hit_left) { /* wait */ }
  analogWrite(RM_PWM, 0);
  analogWrite(LM_PWM, 0);
  Serial.println("Left limit reached");

  delay(500);

  // Move down until BOTTOM limit hit
  digitalWrite(RM_DIR, HIGH); // Right CCW
  digitalWrite(LM_DIR, LOW);  // Left CW
  analogWrite(RM_PWM, 150);
  analogWrite(LM_PWM, 150);
  while (!hit_bottom) { /* wait */ }
  analogWrite(RM_PWM, 0);
  analogWrite(LM_PWM, 0);
  Serial.println("Bottom limit reached");

  encoderCountL = 0;
  encoderCountR = 0;

  Serial.println("Homing complete.");
}

// =================== MAIN ===================
int main() {
  init();
  Serial.begin(9600);

  // ====== Init ======
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);

  analogWrite(LM_PWM, 0);
  analogWrite(RM_PWM, 0);

  // If using pullups, uncomment:
  // pinMode(SW_TOP_PIN,    INPUT_PULLUP);
  // pinMode(SW_RIGHT_PIN,  INPUT_PULLUP);
  // pinMode(SW_LEFT_PIN,   INPUT_PULLUP);
  // pinMode(SW_BOTTOM_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(SW_TOP_PIN),    LimitTop,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_RIGHT_PIN),  LimitRight,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_LEFT_PIN),   LimitLeft,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_BOTTOM_PIN), LimitBottom, CHANGE);

  // Debounce timers (~200 ms windows)
  cli();
  // Timer1 -> X side (LEFT/RIGHT)
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);               // CTC
  TCCR1B |= (1 << CS12) | (1 << CS10);  // prescaler 1024
  OCR1A   = 3125;                       // 16MHz/1024 = 15625 Hz -> 3125 counts ≈ 0.2 s
  TIMSK1 |= (1 << OCIE1A);

  // Timer5 -> Y side (TOP/BOTTOM)
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= (1 << WGM52);               // CTC
  TCCR5B |= (1 << CS52) | (1 << CS50);  // prescaler 1024
  OCR5A   = 3125;
  TIMSK5 |= (1 << OCIE5A);
  sei();

  encoderCountL = 0;
  encoderCountR = 0;
  // ===================

  simulateHoming();

  while (1) { /* idle */ }
  return 0;
}