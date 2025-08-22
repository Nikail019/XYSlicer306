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

// =================== ENCODER COUNTS (optional) ===================
volatile int16_t encoderCountL = 0;
volatile int16_t encoderCountR = 0;

// =================== LIMIT SWITCH ISRs ===================
void LimitTop()    { hit_top    = digitalRead(SW_TOP_PIN); }
void LimitRight()  { hit_right  = digitalRead(SW_RIGHT_PIN); }
void LimitLeft()   { hit_left   = digitalRead(SW_LEFT_PIN); }
void LimitBottom() { hit_bottom = digitalRead(SW_BOTTOM_PIN); }

// =================== INIT ===================
void homingInit() {
  // Motor pins
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);

  analogWrite(LM_PWM, 0);
  analogWrite(RM_PWM, 0);

  // Limit switch pins
//   pinMode(SW_TOP_PIN, INPUT_PULLUP);
//   pinMode(SW_RIGHT_PIN, INPUT_PULLUP);
//   pinMode(SW_LEFT_PIN, INPUT_PULLUP);
//   pinMode(SW_BOTTOM_PIN, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(SW_TOP_PIN),    LimitTop,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_RIGHT_PIN),  LimitRight,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_LEFT_PIN),   LimitLeft,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_BOTTOM_PIN), LimitBottom, CHANGE);

  // Reset encoder counts
  encoderCountL = 0;
  encoderCountR = 0;
}

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

  homingInit();
  simulateHoming();

  while (1) { /* idle */ }
  return 0;
}