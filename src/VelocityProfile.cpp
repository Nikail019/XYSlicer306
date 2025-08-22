// VelocityProfile.cpp  (Arduino Mega 2560)

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <util/atomic.h>



// === Gcode Parsed Values (Need to change in FSM implementation) ===
volatile int   desiredSpeed = 1000;  // enc/min (cruise)
volatile float desiredX     = -1000;  // enc
volatile float desiredY     = 1000;  // enc
volatile float desiredA;  // enc
volatile float desiredB;  // enc

// Limits (mechanical)
float maxSafeAcceleration = 20.0f;   // mm/s^2  (will convert to enc/min^2)
float maxVelocity         = 1347.47f;// mm/min   (will convert to enc/min)
float maxsafePWM          = 255.0f;

// =================== MOTOR PINOUTS ================================
// LEFT motor
const int LM_DIR = 4;       // DIR
const int LM_PWM = 5;       // PWM (Timer3 OC3A)
// RIGHT motor
const int RM_DIR = 7;       // DIR
const int RM_PWM = 6;       // PWM (Timer4 OC4A)

// Encoders (Mega external interrupts ok on 21/20)
const int encoderA_L = 21;
const int encoderB_L = 14;
const int encoderA_R = 20;
const int encoderB_R = 15;

volatile int16_t encoderCountL = 0;
volatile int16_t encoderCountR = 0;

// ====  VELOCITY PROFILE ===========================================
typedef struct {
  volatile float msTime = 0;  // elapsed time in ms
} CurrentMoves;
CurrentMoves moves;

typedef struct {
  float t_acc;  // min
  float d_acc;  // enc
  float d_crs;  // enc
  float t_crs;  // min
} Vprofile;
Vprofile profile;

// ---- Prototypes ----
void   EncoderISR_L();
void   EncoderISR_R();
void   GenerateVProfile();
void   GetTargetVelocity(float* targetVA, float* targetVB);
void   ResetEncoderCounts();
void   PowerMotor(int PWM_L, int PWM_R, int dir_L, int dir_R);
void   SimulateProfile();
float  DistToEncoderCounts(float distance_mm);
void   ABToXY(float* X, float* Y, float A, float B);
void   XYToAB(float X, float Y, float* A, float* B);
static void setupTick1ms();

void ABToXY(float* X, float* Y, float A, float B) {
  *X = (A + B) / 2.0f;
  *Y = (A - B) / 2.0f;
}

void XYToAB(float X, float Y, float* A, float* B) {
  *A = X + Y;
  *B = X - Y;
}

// ============================ MAIN ================================
int main() {
  init();
  cli();

  setupTick1ms();  // Timer2 -> 1ms tick (doesn't touch Timer1/5 or PWM 5/6)

  // Motor pins
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);

  // Encoders
  attachInterrupt(digitalPinToInterrupt(encoderA_L), EncoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_R), EncoderISR_R, RISING);

  sei();

  // Unit conversions
  maxSafeAcceleration = DistToEncoderCounts(maxSafeAcceleration) * 3600.0f; // enc/s^2 -> enc/min^2
  maxVelocity         = DistToEncoderCounts(maxVelocity);                   // mm/min -> enc/min

  Serial.begin(9600);
  ResetEncoderCounts();
  
  SimulateProfile();

  while (1) { /* idle */ }
}

// ========================= Helpers ================================
static void setupTick1ms() {
  // Timer2: 16MHz / 64 = 250kHz. OCR2A=249 -> 1kHz => 1ms period.
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A  = 249;
  TCCR2A |= (1 << WGM21);  // CTC
  TCCR2B |= (1 << CS22);   // prescaler 64
  TIMSK2 |= (1 << OCIE2A); // enable compare A interrupt
}



float DistToEncoderCounts(float distance_mm) {
  // Convert linear distance [mm] -> encoder counts
  const float shaft_diameter_mm     = 12.0f; // pulley/spool diameter
  const int   cpr_before_gear_box   = 12;    // counts per motor rev
  const int   gear_ratio            = 172;
  const int   cpr_after_gear_box    = cpr_before_gear_box * gear_ratio;
  const float counts_to_rad_ratio   = (2.0f * M_PI) / (float)cpr_after_gear_box;
  const float mm_per_count          = 0.5f * counts_to_rad_ratio * shaft_diameter_mm; // r*theta, r=d/2
  return distance_mm / mm_per_count;
}

// ===================== Profile generation =========================
void GenerateVProfile() {
  float a = maxSafeAcceleration; // enc/min^2
  float v = desiredSpeed;        // enc/min

  float x, y;
  ABToXY(&x, &y, desiredA, desiredB);
  float totalmoveXY = sqrtf(x * x + y * y); // enc

  float d_min = (v * v) / a;  // enc

  if (totalmoveXY <= 2.0f * d_min) {
    // Triangular
    Serial.println("Using triangular profile");
    profile.t_acc = sqrtf(totalmoveXY / a); // min
    profile.d_acc = totalmoveXY / 2.0f;
    profile.d_crs = 0.0f;
    profile.t_crs = 0.0f;
  } else {
    // Trapezoidal
    Serial.println("Using trapezoidal profile");
    profile.t_acc = v / a;                          // min
    profile.d_acc = 0.5f * a * profile.t_acc * profile.t_acc;
    profile.d_crs = totalmoveXY - 2.0f * profile.d_acc;
    profile.t_crs = profile.d_crs / v;              // min
  }
}

// XY speed along the profile, split into A/B (enc/min)
void GetTargetVelocity(float* targetVA, float* targetVB) {
  float t = moves.msTime / 60000.0f; // ms -> minutes
  float Vtotal = 0.0f;
  float grad   = (profile.t_acc > 0.0f) ? (desiredSpeed / profile.t_acc) : 0.0f;

  // accel -> cruise -> decel -> stop
  if (t < profile.t_acc) {
    Vtotal = grad * t;
  } else if (t < profile.t_acc + profile.t_crs) {
    Vtotal = desiredSpeed;
  } else if (t < (2.0f * profile.t_acc + profile.t_crs)) {
    float t_dec = t - profile.t_crs - profile.t_acc;
    Vtotal = desiredSpeed - grad * t_dec;
    if (Vtotal < 0.0f) Vtotal = 0.0f;
  } else {
    Vtotal = 0.0f;
  }

  // Split into XY components
  float totalDist = sqrtf(desiredX * desiredX + desiredY * desiredY);
  float Vx = 0.0f, Vy = 0.0f;
  if (totalDist > 1e-6f) {
    Vx = (desiredX / totalDist) * Vtotal;
    Vy = (desiredY / totalDist) * Vtotal;
  }

  // Map XY -> A/B
  XYToAB(Vx, Vy, targetVA, targetVB);
}

// ======================== ISRs ====================================
ISR(TIMER2_COMPA_vect) {
  moves.msTime += 1.0f; // 1 ms tick
}

void EncoderISR_L() {
  if (digitalRead(encoderB_L)) encoderCountL--;
  else                         encoderCountL++;
}
void EncoderISR_R() {
  if (digitalRead(encoderB_R)) encoderCountR--;
  else                         encoderCountR++;
}

// ===================== Motor + Control ============================
void ResetEncoderCounts() {
  encoderCountL = 0;
  encoderCountR = 0;
}

void PowerMotor(int PWM_L, int PWM_R, int dir_L, int dir_R) {
  // dir_* are logic levels (HIGH/LOW), PWM_* are 0..255
  digitalWrite(LM_DIR, dir_L);
  digitalWrite(RM_DIR, dir_R);
  analogWrite(LM_PWM, PWM_L); // pin 5 (Timer3) unaffected by Timer2
  analogWrite(RM_PWM, PWM_R); // pin 6 (Timer4) unaffected by Timer2
}

void SimulateProfile() {
  

  XYToAB(desiredX, desiredY, &desiredA, &desiredB);
  GenerateVProfile();

  Serial.print("max safe acc (enc/min^2): "); Serial.println(maxSafeAcceleration);
  Serial.print("t_acc (min): "); Serial.println(profile.t_acc);
  Serial.print("d_acc (enc): "); Serial.println(profile.d_acc);
  Serial.print("d_crs (enc): "); Serial.println(profile.d_crs);
  Serial.print("t_crs (min): "); Serial.println(profile.t_crs);

  // --- Controller params ---
  const float Kv      = maxsafePWM / maxVelocity; // PWM per (enc/min)
  const float Kp      = 0.10f;                    // PWM per count (tune)

  const int   maxPWM  = (int)maxsafePWM;
  const int   minPWM  = 15;                       // friction deadband
  const uint16_t doneBand = 10;                   // counts/axis
  const uint16_t dt_ms    = 20;                   // control period

  float targetVA = 0.0f, targetVB = 0.0f;

  while(1){
    // Planned target velocities (enc/min)
    GetTargetVelocity(&targetVA, &targetVB);

    // Atomic encoder snapshot
    int16_t a_cnt, b_cnt;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      a_cnt = encoderCountL;
      b_cnt = encoderCountR;
    }

    float errorA = desiredA - (float)a_cnt;
    float errorB = desiredB - (float)b_cnt;

    // ---- DEBUG: encoder counts + errors ----
    Serial.print("Enc L: "); Serial.print(a_cnt);
    Serial.print("  Enc R: "); Serial.print(b_cnt);
    Serial.print("  ErrA: "); Serial.print(errorA);
    Serial.print("  ErrB: "); Serial.println(errorB);

    // Done when inside band and plan is basically stopped 
    //Stop Condition not satisfying

    //it might not work in the other direcition
    if (fabsf(errorA) < 10 && fabsf(errorB) < 10) {
      break;
    }

    // Direction: prefer trajectory, fall back to error sign when near zero
    // int dirA = (fabsf(targetVA) > 1.0f) ? ((targetVA >= 0) ? HIGH : LOW)
    //                                     : ((errorA   >= 0) ? HIGH : LOW);

    // int dirB = (fabsf(targetVB) > 1.0f) ? ((targetVB >= 0) ? HIGH : LOW)
    //                                     : ((errorB   >= 0) ? HIGH : LOW);

    int dirA = (errorA >= 0) ? HIGH : LOW; // always follow the error sign
    int dirB = (errorB >= 0) ? HIGH : LOW; // always follow the error sign
    
    // PWM magnitude: feed-forward + proportional
    int pwmA = (int)(Kv * fabsf(targetVA) + Kp * fabsf(errorA));
    int pwmB = (int)(Kv * fabsf(targetVB) + Kp * fabsf(errorB));

    // Deadband + clamp
    if (pwmA > 0 && pwmA < minPWM) pwmA = minPWM;
    if (pwmB > 0 && pwmB < minPWM) pwmB = minPWM;
    if (pwmA > maxPWM) pwmA = maxPWM;
    if (pwmB > maxPWM) pwmB = maxPWM;

    // ---- DEBUG: PWM + direction being applied each cycle ----
    Serial.print("PWM_A: "); Serial.print(pwmA);
    Serial.print(" DIR_A: "); Serial.print(dirA == HIGH ? "FWD" : "REV");
    Serial.print(" | PWM_B: "); Serial.print(pwmB);
    Serial.print(" DIR_B: "); Serial.println(dirB == HIGH ? "FWD" : "REV");

    PowerMotor(pwmA, pwmB, dirA, dirB);

    _delay_ms(dt_ms);
  }

  Serial.print("outside Loop");

  // stop motors
  PowerMotor(0, 0, LOW, LOW);

  

  Serial.print("enc L: "); Serial.println(encoderCountL);
  Serial.print("enc R: "); Serial.println(encoderCountR);
}