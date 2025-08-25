// VelocityProfile.cpp  (Arduino Mega 2560)

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <util/atomic.h>

// === Gcode Parsed Values (Need to change in FSM implementation) ===
volatile int   desiredSpeed = 1000;   // enc/min (cruise)
volatile float desiredX     = -2000;  // enc
volatile float desiredY     = 0;      // enc
volatile float desiredA;              // enc
volatile float desiredB;              // enc

// Limits (mechanical / electrical)
// NOTE: maxSafeAcceleration is given in mm/s^2 here and converted to enc/min^2 in main()
float maxSafeAcceleration = 1.0f;     // mm/s^2  (will convert to enc/min^2 in main)
float maxVelocity         = 1347.47f; // mm/min   (will convert to enc/min in main)
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
void   GenerateVProfile(); // computes time/distance of triangle/trapezoid (minutes & encoder counts)
void   GetTargetVelocity(float* targetVA, float* targetVB); // time-based profile -> XY -> AB
void   ResetEncoderCounts();
void   PowerMotor(int PWM_L, int PWM_R, int dir_L, int dir_R);
void   SimulateProfile();
float  DistToEncoderCounts(float distance_mm);
void   ABToXY(float* X, float* Y, float A, float B);
void   XYToAB(float X, float Y, float* A, float* B);
static void setupTick1ms();

// CSV logger (PRINTING ONLY changes: expanded to include per-term contributions)
void writeOutputsCSV(
  unsigned long time_ms,
  int16_t encL, int16_t encR,
  float refA, float refB,
  float errA, float errB,
  float tVA, float tVB,
  float ffA, float pA, float iAterm, float dAterm,
  float ffB, float pB, float iBterm, float dBterm,
  int pwmA, int pwmB
);

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

  // Unit conversions (keep everything in enc/min and enc/min^2 for the profile)
  // maxSafeAcceleration (mm/s^2) -> enc/s^2 -> enc/min^2  ( *3600 )
  maxSafeAcceleration = DistToEncoderCounts(maxSafeAcceleration) * 3600.0f;
  // maxVelocity (mm/min) -> enc/min
  maxVelocity         = DistToEncoderCounts(maxVelocity);

  Serial.begin(9600);
  ResetEncoderCounts();

  Serial.println("Starting in 3s...");
  delay(3000);
  Serial.println("Running profile...");
  RunProfileRealtime();

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

void ABToXY(float* X, float* Y, float A, float B) {
  *X = (A + B) / 2.0f;
  *Y = (A - B) / 2.0f;
}

void XYToAB(float X, float Y, float* A, float* B) {
  *A = X + Y;
  *B = X - Y;
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

// ===================== Time-based target velocity =================
// Phase by time (accel/cruise/decel), then project along desired XY and map to A/B.
// Units: returns enc/min for each motor.
void GetTargetVelocity(float* targetVA, float* targetVB) {
  // elapsed time in minutes from the 1ms timebase
  float t_min;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t_min = (float)moves.msTime / 60000.0f;
  }

  // Scalar XY speed from time-profile
  float Vtotal;                         // enc/min
  const float v_cruise = (float)desiredSpeed;
  const float a_lim    = maxSafeAcceleration; // enc/min^2
  const float slope    = (profile.t_acc > 0.0f) ? (v_cruise / profile.t_acc) : 0.0f;

  float totalTime = profile.t_acc + profile.t_crs + profile.t_acc;

  if (t_min <= 0.0f) {
    Vtotal = 0.0f;
  } else if (t_min < profile.t_acc) {
    // accelerating
    Vtotal = slope * t_min;
  } else if (t_min < (profile.t_acc + profile.t_crs)) {
    // cruise
    Vtotal = v_cruise;
  } else if (t_min < totalTime) {
    // decelerating
    float t_dec = t_min - (profile.t_acc + profile.t_crs);
    Vtotal = v_cruise - slope * t_dec;
    if (Vtotal < 0.0f) Vtotal = 0.0f;
  } else {
    Vtotal = 0.0f;
  }

  // Direction of motion from commanded XY vector
  float totalDist = sqrtf(desiredX * desiredX + desiredY * desiredY);
  float ux = 0.0f, uy = 0.0f;
  if (totalDist > 1e-6f) {
    ux = desiredX / totalDist;
    uy = desiredY / totalDist;
  }

  // Split XY scalar speed into components, then map to A/B
  float Vx = ux * Vtotal;   // enc/min
  float Vy = uy * Vtotal;   // enc/min
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

// --- CSV logger for Excel (header printed once) ---
void writeOutputsCSV(
  unsigned long time_ms,
  int16_t encL, int16_t encR,
  float refA, float refB,
  float errA, float errB,
  float tVA, float tVB,
  float ffA, float pA, float iAterm, float dAterm,
  float ffB, float pB, float iBterm, float dBterm,
  int pwmA, int pwmB
) {
  static bool headerPrinted = false;
  if (!headerPrinted) {
    Serial.println(
      "time_ms,"
      "encL,encR,"
      "refA,refB,"
      "errA,errB,"
      "tVA,tVB,"
      "ffA,pA,iAterm,dAterm,"
      "ffB,pB,iBterm,dBterm,"
      "pwmA,pwmB"
    );
    headerPrinted = true;
  }

  Serial.print(time_ms);  Serial.print(',');
  Serial.print(encL);     Serial.print(',');
  Serial.print(encR);     Serial.print(',');

  Serial.print(refA, 3);  Serial.print(',');
  Serial.print(refB, 3);  Serial.print(',');

  Serial.print(errA, 3);  Serial.print(',');
  Serial.print(errB, 3);  Serial.print(',');

  Serial.print(tVA, 3);   Serial.print(',');
  Serial.print(tVB, 3);   Serial.print(',');

  Serial.print(ffA, 3);   Serial.print(',');
  Serial.print(pA, 3);    Serial.print(',');
  Serial.print(iAterm, 3);Serial.print(',');
  Serial.print(dAterm, 3);Serial.print(',');

  Serial.print(ffB, 3);   Serial.print(',');
  Serial.print(pB, 3);    Serial.print(',');
  Serial.print(iBterm, 3);Serial.print(',');
  Serial.print(dBterm, 3);Serial.print(',');

  Serial.print(pwmA);     Serial.print(',');
  Serial.println(pwmB);
}

void SimulateProfile() {
  // Map XY goal -> A/B targets and reset clocks/encoders
  XYToAB(desiredX, desiredY, &desiredA, &desiredB);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    moves.msTime = 0.0f;
    encoderCountL = 0;
    encoderCountR = 0;
  }

  GenerateVProfile();

  // ====== Feed-forward only (no P/I/D) ======
  const float Kv = 0.70f;  // velocity-to-PWM gain (enc/min -> PWM)

  // ====== Timing & limits ======
  const uint16_t dt_ms = 20;
  const int maxPWM    = (int)maxsafePWM;
  const int minPWM    = 25;

  // ====== Stop conditions ======
  const float    posTolCounts   = 10.0f;
  const int      stillTolCounts = 2;
  const uint16_t stillTimeMs    = 600;      // 0.6 s stillness
  const uint16_t stillNeeded    = stillTimeMs / dt_ms;
  const uint32_t safetyMs       = 60000;    // 60 s safety

  int16_t lastEncL = 0, lastEncR = 0;
  uint16_t stillCount = 0;

  float targetVA = 0.0f, targetVB = 0.0f;

  while (1) {
    // 1) Profile velocities (enc/min)
    GetTargetVelocity(&targetVA, &targetVB);

    // 2) Snapshot encoders/time
    int16_t a_cnt, b_cnt;
    unsigned long now_ms;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      a_cnt = encoderCountL;
      b_cnt = encoderCountR;
      now_ms = (unsigned long)moves.msTime;
    }

    // 3) Position errors (for direction and stop checks)
    float errorA = desiredA - (float)a_cnt;
    float errorB = desiredB - (float)b_cnt;

    // 4) Directions (use error sign; magnitudes from Kv only)
    int dirA = (errorA >= 0.0f) ? HIGH : LOW;
    int dirB = (errorB >= 0.0f) ? HIGH : LOW;

    // 5) Pure feed-forward magnitudes
    float ffA = Kv * fabsf(targetVA);
    float ffB = Kv * fabsf(targetVB);

    int pwmA = (int)ffA;
    int pwmB = (int)ffB;

    if (pwmA > 0 && pwmA < minPWM) pwmA = minPWM;
    if (pwmB > 0 && pwmB < minPWM) pwmB = minPWM;
    if (pwmA > maxPWM) pwmA = maxPWM;
    if (pwmB > maxPWM) pwmB = maxPWM;

    // 6) Drive
    PowerMotor(pwmA, pwmB, dirA, dirB);

    // 7) CSV (P/I/D terms shown as 0 by design)
    writeOutputsCSV(
      now_ms,
      a_cnt, b_cnt,
      desiredA, desiredB,
      errorA, errorB,
      targetVA, targetVB,
      ffA, 0.0f, 0.0f, 0.0f,
      ffB, 0.0f, 0.0f, 0.0f,
      pwmA, pwmB
    );

    // 8) Stop condition (position & stillness)
    bool withinPos =
      (fabsf(errorA) <= posTolCounts) &&
      (fabsf(errorB) <= posTolCounts);

    bool noMove =
      (abs(a_cnt - lastEncL) <= stillTolCounts) &&
      (abs(b_cnt - lastEncR) <= stillTolCounts);

    if (withinPos && noMove) {
      if (++stillCount >= stillNeeded) {
        PowerMotor(0, 0, LOW, LOW);
        break;
      }
    } else {
      stillCount = 0;
    }

    lastEncL = a_cnt;
    lastEncR = b_cnt;

    if ((uint32_t)moves.msTime >= safetyMs) {
      PowerMotor(0, 0, LOW, LOW);
      break;
    }

    _delay_ms(dt_ms);
  }

  Serial.println("Profile complete");
  PowerMotor(0, 0, LOW, LOW);
}

void RunProfileRealtime() {
  GenerateVProfile();

  // Reset state for this move
  ResetEncoderCounts();
  moves.msTime = 0;  // ms stopwatch driven by TIMER2 ISR

  const int POS_TOL = 50; // encoder counts tolerance
  float targetVA, targetVB;
  int maxPWM = (int)maxsafePWM;

  // Absolute timeout (ms) for the full trapezoid/triangle
  const unsigned long total_ms = (unsigned long)((profile.t_acc + profile.t_crs + profile.t_acc) * 60000.0f);

  Serial.print("t_acc: "); Serial.println(profile.t_acc);
  Serial.print("d_acc: "); Serial.println(profile.d_acc);
  Serial.print("d_crs: "); Serial.println(profile.d_crs);
  Serial.print("t_crs: "); Serial.println(profile.t_crs);

  // Simple 10 ms control cadence
  while (true) {
    // --- Feedforward target from profile using real elapsed time ---
    GetTargetVelocity(&targetVA, &targetVB);

    // Convert to PWM
    int pwmA = (int)(fabs(targetVA) / maxVelocity * maxPWM);
    int pwmB = (int)(fabs(targetVB) / maxVelocity * maxPWM);
    if (pwmA > maxPWM) pwmA = maxPWM;
    if (pwmB > maxPWM) pwmB = maxPWM;
    if (pwmA < minsafePWM) pwmA = minsafePWM;
    if (pwmB < minsafePWM) pwmB = minsafePWM;

    int dirA = (targetVA >= 0) ? HIGH : LOW;
    int dirB = (targetVB >= 0) ? HIGH : LOW;

    PowerMotor(pwmA, pwmB, dirA, dirB);

    // --- Closed-loop early stop on position tolerance ---
    int16_t encL_snapshot, encR_snapshot;
    uint8_t s = SREG; cli();      // atomic read of 16-bit encoder counts
    encL_snapshot = encoderCountL;
    encR_snapshot = encoderCountR;
    SREG = s;

    long errA = (long)desiredA - (long)encL_snapshot; // A ≙ left
    long errB = (long)desiredB - (long)encR_snapshot; // B ≙ right

    // Telemetry (optional)
    // Serial.print("t_ms="); Serial.print((unsigned long)moves.msTime);
    // Serial.print(" | encL="); Serial.print(encL_snapshot);
    // Serial.print(" | encR="); Serial.print(encR_snapshot);
    // Serial.print(" | errA="); Serial.print(errA);
    // Serial.print(" | errB="); Serial.println(errB);

    // Early stop when both within tolerance
    if (labs(errA) <= POS_TOL && labs(errB) <= POS_TOL) break;

    // Hard stop if profile time exceeded (safety net)
    if ((unsigned long)moves.msTime >= total_ms) break;

    _delay_ms(10);
  }

  PowerMotor(0, 0, LOW, LOW);
  Serial.println("Movement complete (realtime: tol/time).");
}