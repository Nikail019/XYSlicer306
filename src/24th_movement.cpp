// BareBones_Motors_Encoders_Timer.cpp (Arduino Mega 2560)

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

// =================== MOTOR PINOUTS ================================
// LEFT motor
const int LM_DIR = 4;       // DIR
const int LM_PWM = 5;       // PWM (Timer3 OC3A)
// RIGHT motor
const int RM_DIR = 7;       // DIR
const int RM_PWM = 6;       // PWM (Timer4 OC4A)

// =================== ENCODER PINOUTS ==============================
// External interrupts OK on 21/20 (INT0/INT1 on Mega are elsewhere; 21/20 map to INT2/INT3)
const int encoderA_L = 21;  // A channel (interrupt)
const int encoderB_L = 14;  // B channel (digital pin)
const int encoderA_R = 20;  // A channel (interrupt)
const int encoderB_R = 15;  // B channel (digital pin)

// =================== STATE (volatile) =============================
volatile int16_t encoderCountL = 0;
volatile int16_t encoderCountR = 0;
volatile uint32_t tick_ms = 0;  // 1 ms software timebase from Timer2

// =================== PROTOTYPES ===================================
static void setupTick1ms();
static inline void ResetEncoderCounts();
static inline void PowerMotor(int pwmL, int pwmR, int dirL, int dirR);
void EncoderISR_L();
void EncoderISR_R();

// =================== MAIN =========================================
int main() {
  init();     // Arduino core init (GPIO, Timer0 for delay(), etc.)
  cli();      // enter critical while we set timers/ISRs

  // --- 1 ms tick on Timer2 ---
  setupTick1ms();

  // --- Motor pins ---
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);

  // --- Encoder pins ---
  pinMode(encoderB_L, INPUT);        // set as needed: INPUT or INPUT_PULLUP per wiring
  pinMode(encoderB_R, INPUT);
  pinMode(encoderA_L, INPUT);
  pinMode(encoderA_R, INPUT);

  // A channels on rising edge; direction from B channel
  attachInterrupt(digitalPinToInterrupt(encoderA_L), EncoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_R), EncoderISR_R, RISING);

  ResetEncoderCounts();

  sei();      // enable interrupts

  // --- Idle loop (motors off). Adjust as you wish. ---
  PowerMotor(0, 0, LOW, LOW);

    PowerMotor(255, 255, HIGH, HIGH); //ccw
    delay(2000); //move for 2s
    PowerMotor(0, 0, LOW, LOW); //stop
    delay(1000); //wait for 1s
  while (true) {
  }
  // not reached
}

// =================== TIMER2: 1 ms TICK ============================
static void setupTick1ms() {
  // 16 MHz / 64 = 250 kHz. OCR2A = 249 -> 1 kHz => 1 ms period.
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A  = 249;
  TCCR2A |= (1 << WGM21);  // CTC mode
  TCCR2B |= (1 << CS22);   // prescaler 64
  TIMSK2 |= (1 << OCIE2A); // enable compare A interrupt
}

ISR(TIMER2_COMPA_vect) {
  tick_ms++;               // increments every 1 ms
}

// =================== ENCODER ISRs ================================
void EncoderISR_L() {
  // Quadrature: read B to decide direction
  if (digitalRead(encoderB_L)) encoderCountL--;
  else                          encoderCountL++;
}

void EncoderISR_R() {
  if (digitalRead(encoderB_R)) encoderCountR--;
  else                          encoderCountR++;
}

// =================== HELPERS =====================================
static inline void ResetEncoderCounts() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoderCountL = 0;
    encoderCountR = 0;
  }
}

static inline void PowerMotor(int pwmL, int pwmR, int dirL, int dirR) {
  // dirL/dirR are logic levels (HIGH/LOW). pwmL/pwmR in [0..255].
  digitalWrite(LM_DIR, dirL);
  digitalWrite(RM_DIR, dirR);
  analogWrite(LM_PWM, pwmL);  // Timer3
  analogWrite(RM_PWM, pwmR);  // Timer4
}

float CalculateSpeed(int pwm) {
  // Simple linear model: speed (mm/s) = k * pwm
  const float k = 0.5; // mm/s per PWM unit, adjust as needed
  return k * pwm;
}

//measuring speed