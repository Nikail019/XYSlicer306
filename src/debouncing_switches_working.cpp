#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>

volatile bool x_debounce_flag = false;  // X-axis (RIGHT/LEFT)
volatile bool y_debounce_flag = false;  // Y-axis (TOP/BOTTOM)
volatile bool hit_top = false;
volatile bool hit_right = false;
volatile bool hit_left = false;
volatile bool hit_bottom = false;

volatile int E1 = 0;  // Left Encoder Value
volatile int E2 = 0;  // Right Encoder Value

// ---- pick your debounce time here ----
// 16 MHz / 1024 = 15625 ticks/sec
// OCRnA = debounce_s * 15625 - 1
// e.g. 50 ms  => ~781; 200 ms => ~3124
constexpr uint16_t X_DEBOUNCE_OCR = 3124;  // ~200 ms for X
constexpr uint16_t Y_DEBOUNCE_OCR = 3124;  // ~200 ms for Y

int main() {
  Serial.begin(9600);
  cli();

  // TIMER1 -> X debounce (CTC, prescaler 1024)
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);                 // CTC
  TCCR1B |= (1 << CS12) | (1 << CS10);    // /1024
  OCR1A = X_DEBOUNCE_OCR;
  TIMSK1 |= (1 << OCIE1A);                // enable compare A IRQ

  // TIMER3 -> Y debounce (CTC, prescaler 1024)
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B |= (1 << WGM32);                 // CTC
  TCCR3B |= (1 << CS32) | (1 << CS30);    // /1024
  OCR3A = Y_DEBOUNCE_OCR;
  TIMSK3 |= (1 << OCIE3A);                // enable compare A IRQ

  // LIMIT SWITCH INTERRUPT SETUP
  // NOTE: Mega external interrupts are on 2, 3, 18, 19, 20, 21.
  attachInterrupt(digitalPinToInterrupt(2),  LimitTop,    RISING);   // change as needed
  attachInterrupt(digitalPinToInterrupt(3),  LimitRight,  RISING);
  attachInterrupt(digitalPinToInterrupt(19), LimitLeft,   RISING);
  attachInterrupt(digitalPinToInterrupt(18), LimitBottom, RISING);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(19, INPUT);
  pinMode(18, INPUT);

  // no pullups if your switches already provide clean signals
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(19, LOW);
  digitalWrite(18, LOW);

  sei();

  while (1) {
    // your main loop
  }
}

// ---- ISRs for limit switches ----
// Y-axis: TOP/BOTTOM -> reset Y timer (TIMER3) and set Y debounce
void LimitTop() {
  if (!y_debounce_flag) {
    Serial.write("TOP \n");
    hit_top = true;
    y_debounce_flag = true;
    TCNT3 = 0;               // restart Y debounce timing
  }
}

void LimitBottom() {
  if (!y_debounce_flag) {
    Serial.write("BOTTOM \n");
    hit_bottom = true;
    y_debounce_flag = true;
    TCNT3 = 0;               // restart Y debounce timing
  }
}

// X-axis: RIGHT/LEFT -> reset X timer (TIMER1) and set X debounce
void LimitRight() {
  if (!x_debounce_flag) {
    Serial.write("RIGHT \n");
    hit_right = true;
    x_debounce_flag = true;
    TCNT1 = 0;               // restart X debounce timing
  }
}

void LimitLeft() {
  if (!x_debounce_flag) {
    Serial.write("LEFT \n");
    hit_left = true;
    x_debounce_flag = true;
    TCNT1 = 0;               // restart X debounce timing
  }
}

// ---- Timer ISRs ----
// Only clear the matching axis flag in each ISR
ISR(TIMER1_COMPA_vect) {     // X debounce done
  x_debounce_flag = false;
}

ISR(TIMER3_COMPA_vect) {     // Y debounce done
  y_debounce_flag = false;
}
