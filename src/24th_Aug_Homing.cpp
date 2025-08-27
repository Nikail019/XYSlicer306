#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>

// ===================== MECHANICAL NOTES (informational) =====================
// Pulley dia 12 mm -> circumference ≈ 37.699 mm.
// Encoders: 12 ticks/rev @ motor, 172:1 gear -> 2064 ticks/output rev.
// Distance per tick ≈ 0.018265 mm. Kept here as comments for later use.
// ===========================================================================

// === Gcode Parsed Values (all declared directly in mm and mm/s) ===
// Positions in millimetres (mm)
volatile float desiredX = 200.0f;   // mm
volatile float desiredY =  0.0f;   // mm

// Velocity in millimetres per second (mm/s)
volatile float desiredSpeed = 20.0f;   // mm/s

// Limits directly in mm/s and mm/s^2
float maxSafeAcceleration = 75.0f;     // mm/s^2
float maxVelocity         = 35.0f;     // mm/s
float maxsafePWM          = 178;       // (dimensionless PWM)

// =================== MOTOR PINOUTS (COPIED 13/08/25) =======================
const int LM_DIR = 4;       // Left Motor direction pin
const int LM_PWM = 5;       // Left Motor PWM pin
const int encoderA_L = 21;  // Channel A (external interrupt pin)
const int encoderB_L = 14;  // Channel B (digital pin)
volatile int16_t encoderCountL = 0;

const int RM_DIR = 7;       // Right Motor direction pin
const int RM_PWM = 6;       // Right Motor PWM pin
const int encoderA_R = 20;  // Channel A (external interrupt pin)
const int encoderB_R = 15;  // Channel B (digital pin)
volatile int16_t encoderCountR = 0;
// ===========================================================================

// ====  VELOCITY PROFILE AND CONTROL SETUP ==================================
typedef struct {
  volatile float msTime = 0;  // elapsed time in ms
} CurrentMoves;
CurrentMoves moves;

typedef struct {
  // velocity profile along XY path
  float t_acc;  // s
  float d_acc;  // mm
  float d_crs;  // mm
  float t_crs;  // s
} Vprofile;
Vprofile profile;
// ===========================================================================

// ---------------------- Forward Declarations ------------------------
void SimulateProfile();
void GenerateVProfile();
void GetTargetVelocity(float* targetVA, float* targetVB); // mm/s in A/B
void GetTargetDistance(float* distA, float* distB);       // mm in A/B
void XYToAB(float X, float Y, float* A, float* B);
// -------------------------------------------------------------------

int main() {
  cli();
  /*TIMER 4 (PID) SETUP*/
  TCNT4 = 0;                            // initalize counter to 0
  OCR4A = 249;                          // 250 counts = 1ms
  TCCR4B |= (1 << WGM42);               // CTC
  TCCR4B |= (1 << CS41) | (1 << CS40);  // Prescaler = 64;
  TIMSK4 |= (1 << OCIE4A);              // Enable timer 4 compare A interrupt

  /*MOTOR SETUP*/
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);

  sei();

  Serial.begin(9600);
  SimulateProfile();

  while (1) {
  }
}

// Convert XY to A/B (A = X+Y, B = X−Y), units preserved (mm or mm/s)
void XYToAB(float X, float Y, float* A, float* B) {
  *A = X + Y;
  *B = X - Y;
}

// Build trapezoidal / triangular profile along XY distance
void GenerateVProfile() {
  float a = maxSafeAcceleration; // mm/s^2
  float v = desiredSpeed;        // mm/s

  // Clamp to avoid div by zero / nonsense
  if (a <= 0.0f) a = 1e-6f;
  if (v <  0.0f) v = 0.0f;

  const float L = sqrtf(desiredX*desiredX + desiredY*desiredY); // path length (mm)
  if (L <= 0.0f || v <= 0.0f) {
    profile.t_acc = 0.0f;
    profile.d_acc = 0.0f;
    profile.d_crs = 0.0f;
    profile.t_crs = 0.0f;
    Serial.println("Degenerate move or zero speed; profile empty.");
    return;
  }

  const float d_min = (v * v) / a;  // mm

  if (L <= 2.0f * d_min) {
    Serial.println("Using triangular profile");
    profile.t_acc = sqrtf(L / a); // s
    profile.d_acc = L / 2.0f;     // mm
    profile.d_crs = 0.0f;         // mm
    profile.t_crs = 0.0f;         // s
  } else {
    Serial.println("Using trapezoidal profile");
    profile.t_acc = v / a;                                     // s
    profile.d_acc = 0.5f * a * profile.t_acc * profile.t_acc;  // mm
    profile.d_crs = L - 2.0f * profile.d_acc;                  // mm
    profile.t_crs = profile.d_crs / v;                         // s
  }
}

// Target velocity (A/B) by projecting the XY magnitude onto the path direction
void GetTargetVelocity(float* targetVA, float* targetVB) {
  const float t = moves.msTime / 1000.0f; // s
  const float L = sqrtf(desiredX*desiredX + desiredY*desiredY); // mm
  if (L <= 0.0f) { *targetVA = 0.0f; *targetVB = 0.0f; return; }

  // Unit direction in XY
  const float ux = desiredX / L;
  const float uy = desiredY / L;

  // Piecewise XY speed magnitude
  const float v  = desiredSpeed;                            // mm/s
  const float a  = (profile.t_acc > 0.0f) ? (v / profile.t_acc) : 0.0f; // mm/s^2
  const float t1 = profile.t_acc;
  const float t2 = profile.t_acc + profile.t_crs;

  float Vtotal = 0.0f; // mm/s
  if (t < 0.0f) {
    Vtotal = 0.0f;
  } else if (t < t1) {
    Vtotal = a * t;
  } else if (t < t2) {
    Vtotal = v;
  } else if (t < t2 + profile.t_acc) {
    const float td = t - t2;
    Vtotal = v - a * td;
    if (Vtotal < 0.0f) Vtotal = 0.0f;
  } else {
    Vtotal = 0.0f;
  }

  // XY components, then to A/B
  const float Vx = ux * Vtotal;
  const float Vy = uy * Vtotal;
  XYToAB(Vx, Vy, targetVA, targetVB); // mm/s
}

// Target distance (A/B) by integrating the XY profile and projecting onto axes
void GetTargetDistance(float* distA, float* distB) {
  const float t = moves.msTime / 1000.0f; // s
  const float L = sqrtf(desiredX*desiredX + desiredY*desiredY); // mm
  if (L <= 0.0f) { *distA = 0.0f; *distB = 0.0f; return; }

  const float ux = desiredX / L;
  const float uy = desiredY / L;

  // Segment boundaries and kinematics
  const float v  = desiredSpeed;                           // mm/s
  const float a  = (profile.t_acc > 0.0f) ? (v / profile.t_acc) : 0.0f; // mm/s^2
  const float t1 = profile.t_acc;                   // end accel
  const float t2 = profile.t_acc + profile.t_crs;   // end cruise
  const float t3 = t2 + profile.t_acc;              // end decel

  // Path distance s(t)
  float s = 0.0f; // mm
  if (t <= 0.0f) {
    s = 0.0f;
  } else if (t < t1) {
    s = 0.5f * a * t * t;
  } else if (t < t2) {
    s = profile.d_acc + v * (t - t1);
  } else if (t < t3) {
    const float td = t - t2;
    s = profile.d_acc + v * profile.t_crs + (v * td - 0.5f * a * td * td);
  } else {
    s = L;
  }
  if (s < 0.0f) s = 0.0f;
  if (s > L)    s = L;

  // Resolve along XY then convert to A/B
  const float sX = ux * s;
  const float sY = uy * s;
  XYToAB(sX, sY, distA, distB); // mm
}

ISR(TIMER4_COMPA_vect) {
  // 1 kHz timebase
  moves.msTime++;
}

void ResetEncoderCounts() {
  encoderCountL = 0;
  encoderCountR = 0;
}

void PowerMotor(int PWM_L, int PWM_R, int dir_L, int dir_R) {
  digitalWrite(RM_DIR, dir_R);
  digitalWrite(LM_DIR, dir_L);
  analogWrite(RM_PWM, PWM_R);
  analogWrite(LM_PWM, PWM_L);
}

void SimulateProfile() {
  GenerateVProfile();

  // Print profile summary
  Serial.print("t_acc (s): ");  Serial.println(profile.t_acc, 6);
  Serial.print("d_acc (mm): "); Serial.println(profile.d_acc, 6);
  Serial.print("d_crs (mm): "); Serial.println(profile.d_crs, 6);
  Serial.print("t_crs (s): ");  Serial.println(profile.t_crs, 6);

  float DA, DB; // distances in mm along A/B
  // float VA, VB; // (optional) velocities in mm/s along A/B

  // Simulate for total duration
  const float totalTime = profile.t_acc + profile.t_crs + profile.t_acc; // s
  for (float t = 0.0f; t <= totalTime; t += 0.1f) { // 0.1 s steps
    moves.msTime = t * 1000.0f; // ms

    GetTargetDistance(&DA, &DB);
    // GetTargetVelocity(&VA, &VB); // uncomment to log velocities as well

    Serial.print("t (s): ");      Serial.print(t, 2);
    Serial.print(" | DA (mm): "); Serial.print(DA, 3);
    Serial.print(" | DB (mm): "); Serial.print(DB, 3);

    // Also show equivalent XY distance projection (helpful for sanity check)
    const float L = sqrtf(desiredX*desiredX + desiredY*desiredY);
    const float ux = (L > 0.0f) ? desiredX / L : 0.0f;
    const float uy = (L > 0.0f) ? desiredY / L : 0.0f;

    // s = |(sX,sY)|; recover s from A/B: s = 0.5*sqrt((A+B)^2 + (A−B)^2)
    const float s = 0.5f * sqrtf((DA + DB)*(DA + DB) + (DA - DB)*(DA - DB));
    const float sX = ux * s;
    const float sY = uy * s;

    Serial.print(" | sX (mm): "); Serial.print(sX, 3);
    Serial.print(" | sY (mm): "); Serial.println(sY, 3);
  }
}
