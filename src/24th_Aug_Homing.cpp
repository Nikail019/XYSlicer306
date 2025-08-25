#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
// CHECK ALLOW LIBRARIES:
#include <stdlib.h>
#include <string.h>
#include <math.h>

struct VProfile;  // forward-declare so Arduino's auto-prototypes know the type
// =================== MOTOR PINOUTS (COPIED 13/08/25) ==============
// === LEFT MOTOR ===
const int LM_DIR   = 4;    // Left Motor direction pin
const int LM_PWM   = 5;    // Left Motor PWM pin
const int encoderA_L = 21; // Channel A (external interrupt pin)
const int encoderB_L = 14; // Channel B (digital pin)

volatile int16_t encoderCountL = 0;

// === RIGHT MOTOR ===
const int RM_DIR   = 7;    // Right Motor direction pin
const int RM_PWM   = 6;    // Right Motor PWM pin
const int encoderA_R = 20; // Channel A (external interrupt pin)
const int encoderB_R = 15; // Channel B (digital pin)

volatile int16_t encoderCountR = 0;
// ======= TUNABLES =======
static const int BASE_PWM   = 180;  // starting drive; set just above static friction
static const int PWM_MIN    = 35;  // keep some torque in the slow motor
static const int PWM_MAX    = 255;
static const int KP_NUM     = 6;    // proportional gain (num/den) for balance
static const int KP_DEN     = 10;   // e.g., 0.6
static const int L_PWM_ADJ  = 0;    // empirical trim
static const int R_PWM_ADJ  = 3;    // empirical trim

// ===== Velocity-profile + per-motor P control (CoreXY) =====

// Feedforward scaling & limits
static const float MAX_V_MM_MIN   = 1000.0f;   // your maxVelocity (mm/min)
static const float MAX_A_MM_MIN2  = 1000.0f;   // your maxSafeAcceleration (mm/min^2)
static const int   FF_PWM_MAX     = 178;       // your maxsafePWM
static const int   FF_PWM_MIN     = 30;        // your minsafePWM

// Feedback (P) tuning
static const float KP_PWM_PER_MM  = 6.0f;      // try 3–10; raises/reduces correction strength
static const float STOP_EPS_MM    = 0.2f;      // how close to target to consider "done"
static const uint16_t LOOP_DT_MS  = 10;        // control loop period (ms)



// Simple CoreXY helpers
static inline void ABToXY(float* X, float* Y, float A, float B) {
  *X = (A + B) * 0.5f;
  *Y = (A - B) * 0.5f;
}
static inline void XYToAB(float X, float Y, float* A, float* B) {
  *A = X + Y;
  *B = X - Y;
}

// Trapezoid/triangular profile
struct VProfile {
  float t_acc;  // min
  float d_acc;  // mm
  float d_crs;  // mm
  float t_crs;  // min
};

// Compute profile for a straight-line move of length totalXY (mm)
static inline VProfile GenProfileXY(float totalXY_mm, float v_cruise_mm_min, float a_mm_min2) {
  VProfile p{};
  const float d_min = (v_cruise_mm_min * v_cruise_mm_min) / a_mm_min2;
  if (totalXY_mm <= 2.0f * d_min) {
    // triangular
    p.t_acc = sqrtf(totalXY_mm / a_mm_min2);
    p.d_acc = 0.5f * totalXY_mm;
    p.d_crs = 0.0f;
    p.t_crs = 0.0f;
  } else {
    // trapezoidal
    p.t_acc = v_cruise_mm_min / a_mm_min2;
    p.d_acc = 0.5f * a_mm_min2 * p.t_acc * p.t_acc;
    p.d_crs = totalXY_mm - 2.0f * p.d_acc;
    p.t_crs = p.d_crs / v_cruise_mm_min;
  }
  return p;
}

// Scalar target speed along the path at time t (minutes)
static inline float ProfileSpeedAt(const VProfile& p, float v_cruise, float t_min) {
  const float slope = v_cruise / p.t_acc;
  if (t_min < p.t_acc) {
    return slope * t_min;
  }
  if (t_min > (p.t_acc + p.t_crs)) {
    const float t_dec = t_min - p.t_acc - p.t_crs;
    return v_cruise - slope * t_dec;
  }
  return v_cruise;
}

// Convenience: signed PWM write for one motor (dir + magnitude)
static inline void driveLeft(float u_pwm_signed) {
  int dir = (u_pwm_signed >= 0) ? HIGH : LOW;
  int mag = (int)fabsf(u_pwm_signed);
  if (mag > PWM_MAX) mag = PWM_MAX;      // you already have PWM_MAX=255 in your tunables
  if (mag < 0) mag = 0;
  digitalWrite(LM_DIR, dir);
  analogWrite(LM_PWM, mag);
}
static inline void driveRight(float u_pwm_signed) {
  int dir = (u_pwm_signed >= 0) ? HIGH : LOW;
  int mag = (int)fabsf(u_pwm_signed);
  if (mag > PWM_MAX) mag = PWM_MAX;
  if (mag < 0) mag = 0;
  digitalWrite(RM_DIR, dir);
  analogWrite(RM_PWM, mag);
}

// Read current A/B distances (mm) relative to a given start count
static inline void getAB_mm_fromEnc(int16_t startL, int16_t startR, float* A_mm, float* B_mm) {
  int16_t dL = encoderCountL - startL;
  int16_t dR = encoderCountR - startR;
  *A_mm = EncoderCountsToDist((uint16_t)dL);
  *B_mm = EncoderCountsToDist((uint16_t)dR);
}

// ===============================================================

// =================== FSM/GCODE =================================
volatile int state;

/* STATE DEFINTIONS*/
#define IDLE 0
#define PARSE 1
#define MOVE 2
#define HOME 3
#define FAULT 4

/* GCODE PARSING*/
#define M999 5
#define INVALID 6  // Command does not exist
#define INPUT_BUFFER_SIZE 64
volatile char commandStr[INPUT_BUFFER_SIZE];

volatile int desiredSpeed = 0; // in mm/min
volatile int Xdist; // in mm
volatile int Ydist; // in mm

// ===============================================================

/*LIMIT SWITCH SETUP*/
volatile bool x_debounce_flag = false;
volatile bool y_debounce_flag = false;
volatile bool hit_top = false;
volatile bool hit_right = false;
volatile bool hit_left = false;
volatile bool hit_bottom = false;

// ===============================================================
inline bool abortIfFault(const char* where){
  if (state == FAULT){
    analogWrite(RM_PWM, 0); analogWrite(LM_PWM, 0);
    Serial.print("Homing aborted due to FAULT ("); Serial.print(where); Serial.println(").");
    return true;
  }
  return false;
}

int main() {
  init();
  Serial.begin(9600);

  cli();
  /*TIMER SETUP*/
  // timer counter 1 with prescaler of 1024
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TCCR1B |= (1 << WGM12);  // ctc mode
  OCR1A = 3125;
  // --- CHANGED: use timer counter 5 (instead of 3) with prescaler of 1024
  TCCR5B |= (1 << CS52) | (1 << CS50);
  TCCR5B |= (1 << WGM52);  // ctc mode
  OCR5A = 3125;

  TIMSK1 |= (1 << OCIE1A);  // ISR on Compare Match
  // --- CHANGED: enable Timer5 compare A interrupt
  TIMSK5 |= (1 << OCIE5A);

  /*LIMIT SWITCH SETUP*/ //Changed 
  //  TOP, RIGHT, LEFT, BOTTOM == D2, D3, D19, D18

  //changed from RISING to CHANGE
  attachInterrupt(digitalPinToInterrupt(2), LimitTop, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), LimitRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), LimitLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), LimitBottom, CHANGE);

  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(encoderA_L), EncoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_R), EncoderISR_R, RISING);


  // ================== Motor Output ============//
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);

// ============================================
  sei();

  newState(IDLE);  // Initally in the IDLE state

  while (1) {
    switch (state) {
      case IDLE:
        if (checkForCommand()) {
          newState(PARSE);
        }
        break;

      case PARSE:
        parseCommand();
        break;

      case MOVE:
        simulateMovement();
        break;

      case HOME:
        simulateHoming();
        break;

      case FAULT:
        digitalWrite(LM_DIR, LOW);
        digitalWrite(RM_DIR, LOW);
        analogWrite(LM_PWM, 0);
        analogWrite(RM_PWM, 0);
        if (checkForCommand()) {
          parseErrorCommand();
        }
        break;
    }
  }
  return 0;
}

void newState(int ns) {
  state = ns;
  reportState();
}

void parseCommand() {
  // Used in the IDLE state to check for valid commands. Changes to
  // corresponding state
   getCommand();

  int commandType = checkCommandType();
  // Change state depending on command type
  if (commandType == HOME) {
    newState(HOME);

  } else if (commandType == INVALID || commandType == M999) {
    // M999 is invalid in this state
    reportError("Command is unrecognised or invalid");
    newState(FAULT);

  } else if (commandType == MOVE) {
    // Extra checks for validty of G01/MOVE command. Logic is missing to extract
    // X, Y and speed from command.
    if (!checkMovementCommand()) {
      reportError("Movement command is invalid or missing elements");
      newState(FAULT);
    } else {
      getG01Values(); //Gets speed and X/Y distances fr
      newState(MOVE);
    }
  }
}

void parseErrorCommand() {
  // Used in the FAULT state to check ONLY for the reset to idle (M999) command
  getCommand();

  int commandType = checkCommandType();
  if (commandType == M999) {
    newState(IDLE);
    return;

  } else if (commandType == INVALID) {
    reportError("Command is unrecognised");
  } else if (commandType == MOVE || commandType == HOME) {
    reportError("This type of command is invalid in the error state \n");
  }
  return;
}

bool checkForCommand() {
  // Checks for any serial input
  if (Serial.available() > 0) {
    return true;
  } else {
    return false;
  }
}

int checkCommandType() {
  // Checks the first block of the command for its type (G01, G28, M999) and
  // returns the related state, command (for M999) otherwise "INVALID"
  if (strstr(commandStr, "M999") != NULL ||
      strstr(commandStr, "m999") != NULL) {
    return M999;
  } else if (strstr(commandStr, "G01") != NULL ||
             strstr(commandStr, "g01") != NULL ||
             strstr(commandStr, "g1") != NULL ||
             strstr(commandStr, "G1") != NULL) {
    return MOVE;
  } else if (strstr(commandStr, "G28") != NULL ||
             strstr(commandStr, "g28") != NULL) {
    return HOME;
  } else {
    return INVALID;
  }
}

bool checkMovementCommand() {
  // Checks if movement command is valid bt checking the following components:
  // ( X component AND Y component ) AND (F component OR already existing speed)
  if ((strstr(commandStr, "X") != NULL && strstr(commandStr, "Y") != NULL) &&
      (desiredSpeed != 0 || strstr(commandStr, "F") != NULL)) {
    return true;
  } else {
    return false;
  }
}

void simulateHoming() {
  // Reset flags before starting
  hit_top = hit_right = hit_left = hit_bottom = false;
  x_debounce_flag = y_debounce_flag = false;

  Serial.println("Homing BRuv");

  // ========= PHASE 1: Move LEFT (pure X-)
  {
    Serial.println("Moving left to hit left limit switch...");

    const bool dirL = LOW; 
    const bool dirR = LOW;

    int16_t startL = encoderCountL;
    int16_t startR = encoderCountR;

    // prime outputs
    digitalWrite(LM_DIR, dirL);
    digitalWrite(RM_DIR, dirR);
    analogWrite(LM_PWM, PWM_MIN+L_PWM_ADJ);
    analogWrite(RM_PWM, PWM_MIN+R_PWM_ADJ);

    // Wait until left limit hit or FAULT
    while (!hit_left && state != FAULT) {
    }

    // Abort on fault
    if (abortIfFault("during left-seek")) return;

    // Stop and small settle
    analogWrite(LM_PWM, 0);
    analogWrite(RM_PWM, 0);
    Serial.println("Left limit switch hit, stopping motors.");
    delay(150);
  }

  // ========= PHASE 2: Move DOWN (pure Y-)
  {
    Serial.println("Moving down to hit bottom limit switch...");
    const bool dirL = LOW;  
    const bool dirR = HIGH;

    int16_t startL = encoderCountL;
    int16_t startR = encoderCountR;

    digitalWrite(LM_DIR, dirL);
    digitalWrite(RM_DIR, dirR);
    analogWrite(LM_PWM, PWM_MIN + L_PWM_ADJ);
    analogWrite(RM_PWM, PWM_MIN + R_PWM_ADJ);

    while (!hit_bottom && state != FAULT) {
    }

    if (abortIfFault("during bottom-seek")) return;

    analogWrite(LM_PWM, 0);
    analogWrite(RM_PWM, 0);
    Serial.println("Bottom limit switch hit, stopping motors.");
  }

  // Zero encoders at home
  encoderCountL = 0;
  encoderCountR = 0;

  Serial.println("Homing complete.");
  newState(IDLE);
}


void simulateMovement() {
  // ===== CSV header (once per move) =====
  Serial.println(F("ms,exp_s_mm,total_mm,Vtot,VA,VB,A_mm,B_mm,L_cnt,R_cnt,PWM_A,PWM_B"));

  // Uses desiredSpeed (mm/min), Xdist (mm), Ydist (mm) from parser
  float totalXY = sqrtf((float)Xdist * Xdist + (float)Ydist * Ydist);
  if (totalXY < 1e-3f) {
    Serial.println("Move skipped (zero length).");
    newState(IDLE);
    return;
  }
  const float ux = (float)Xdist / totalXY;
  const float uy = (float)Ydist / totalXY;

  const float v_cruise = constrain((float)desiredSpeed, 1.0f, MAX_V_MM_MIN);
  const float a_used   = MAX_A_MM_MIN2;

  VProfile prof = GenProfileXY(totalXY, v_cruise, a_used);

  Serial.println("=== MOVE BEGIN ===");
  Serial.print("totalXY(mm): "); Serial.println(totalXY, 3);
  Serial.print("v_cruise(mm/min): "); Serial.println(v_cruise, 2);
  Serial.print("t_acc(min): "); Serial.println(prof.t_acc, 4);
  Serial.print("d_acc(mm): "); Serial.println(prof.d_acc, 3);
  Serial.print("d_crs(mm): "); Serial.println(prof.d_crs, 3);
  Serial.print("t_crs(min): "); Serial.println(prof.t_crs, 4);

  const float total_time_min = prof.t_acc + prof.t_crs + prof.t_acc;
  const unsigned long HARD_STOP_MS =
      (unsigned long)(total_time_min * 60000.0f) + 100UL; // +100ms guard

  const unsigned long t0 = millis();
  const int16_t L0 = encoderCountL;
  const int16_t R0 = encoderCountR;

  // expected along-track and per-motor (mm)
  float exp_s_mm = 0.0f;
  float expA_mm = 0.0f, expB_mm = 0.0f;

  unsigned long prev_ms = t0;

  // throttle serial: print every LOG_DIV control ticks
  const uint8_t LOG_DIV = 10; // LOOP_DT_MS=10ms -> ~100ms logging
  uint16_t log_i = 0;

  while (true) {
    if (state == FAULT) {
      reportError("FAULT during MOVE — stopping.");
      driveLeft(0); driveRight(0);
      return;
    }

    const unsigned long now_ms = millis();
    const unsigned long dt_ms  = now_ms - prev_ms;
    prev_ms = now_ms;

    const unsigned long elapsed_ms = now_ms - t0;
    float t_min = elapsed_ms / 60000.0f;
    if (t_min > total_time_min) t_min = total_time_min;

    // scalar path speed from profile
    const float Vtot = ProfileSpeedAt(prof, v_cruise, t_min); // mm/min

    // integrate expected along-track distance (mm)
    exp_s_mm += Vtot * (dt_ms / 60000.0f);
    if (exp_s_mm > totalXY) exp_s_mm = totalXY;

    // expected XY and CoreXY A/B (mm)
    const float expX = ux * exp_s_mm;
    const float expY = uy * exp_s_mm;
    XYToAB(expX, expY, &expA_mm, &expB_mm);

    // actual A/B (mm) from encoders
    float A_mm = 0.0f, B_mm = 0.0f;
    getAB_mm_fromEnc(L0, R0, &A_mm, &B_mm);

    // target A/B velocities (mm/min) from path
    float Vx = ux * Vtot, Vy = uy * Vtot;
    float VA, VB; XYToAB(Vx, Vy, &VA, &VB);

    // P control around signed feedforward
    const float speed_scale = (v_cruise > 1.0f) ? fabsf(Vtot) / v_cruise : 0.0f;
    float eA = expA_mm - A_mm;
    float eB = expB_mm - B_mm;
    float ffA = (VA / MAX_V_MM_MIN) * FF_PWM_MAX;   // signed
    float ffB = (VB / MAX_V_MM_MIN) * FF_PWM_MAX;   // signed
    float uA  = ffA + (KP_PWM_PER_MM * speed_scale) * eA;  // signed PWM cmd
    float uB  = ffB + (KP_PWM_PER_MM * speed_scale) * eB;

    auto enforce_min_when_moving = [&](float u_signed) -> float {
      float mag = fabsf(u_signed);
      if (fabsf(Vtot) < 1.0f) return 0.0f;  // profile says "stop" -> zero output
      if (mag < FF_PWM_MIN) mag = FF_PWM_MIN;
      if (mag > FF_PWM_MAX) mag = FF_PWM_MAX;
      return (u_signed >= 0) ? mag : -mag;
    };
    float uA_applied = enforce_min_when_moving(uA);
    float uB_applied = enforce_min_when_moving(uB);

    // ====== SERIAL LOG (CSV) ======
    if ((++log_i % LOG_DIV) == 0) {
      // snapshot raw counts (non-atomic is fine for debug; optional to guard)
      int16_t Lcnt = encoderCountL;
      int16_t Rcnt = encoderCountR;

      Serial.print(elapsed_ms);           Serial.print(',');
      Serial.print(exp_s_mm, 3);          Serial.print(',');
      Serial.print(totalXY, 3);           Serial.print(',');
      Serial.print(Vtot, 2);              Serial.print(',');
      Serial.print(VA, 2);                Serial.print(',');
      Serial.print(VB, 2);                Serial.print(',');
      Serial.print(A_mm, 3);              Serial.print(',');
      Serial.print(B_mm, 3);              Serial.print(',');
      Serial.print(Lcnt);                 Serial.print(',');
      Serial.print(Rcnt);                 Serial.print(',');
      Serial.print(uA_applied, 1);        Serial.print(',');
      Serial.println(uB_applied, 1);
    }

    // stop conditions
    const bool time_done   = (elapsed_ms >= HARD_STOP_MS);
    const bool expect_done = ((totalXY - exp_s_mm) <= STOP_EPS_MM);
    if (time_done || expect_done) {
      driveLeft(0); driveRight(0);
      Serial.println("=== MOVE COMPLETE ===");
      newState(IDLE);
      return;
    }

    // drive motors
    driveLeft(uA_applied);
    driveRight(uB_applied);

    delay(LOOP_DT_MS);
  }
}


void reportError(const char *errorCode) { Serial.println(errorCode); }

void reportState() {
  Serial.print("State is: ");
  Serial.println(state);
}

// ==================== LIMIT SWITCH ISRs ====================

// LEFT limit switch
void LimitLeft() {
  if (state == HOME) {
    // Normal stop during homing
    TCNT1 = 0;
    hit_left = true;
    x_debounce_flag = true;
    return;
  }

  // Any other state = FAULT
  if (!x_debounce_flag) {
    reportError("Left limit switch triggered unexpectedly.");
    newState(FAULT);
    TCNT1 = 0;
    hit_left = true;
    x_debounce_flag = true;
  }
}

// RIGHT limit switch
void LimitRight() {
  if (state == HOME) {
    // Should never happen in homing
    reportError("Right limit switch triggered during homing!");
    newState(FAULT);
    return;
  }

  if (!x_debounce_flag) {
    reportError("Right limit switch triggered.");
    newState(FAULT);
    TCNT1 = 0;
    hit_right = true;
    x_debounce_flag = true;
  }
}

// BOTTOM limit switch
void LimitBottom() {
  if (state == HOME) {
    // Normal stop during homing
    TCNT5 = 0;
    hit_bottom = true;
    y_debounce_flag = true;
    return;
  }

  if (!y_debounce_flag) {
    reportError("Bottom limit switch triggered unexpectedly.");
    newState(FAULT);
    TCNT5 = 0;
    hit_bottom = true;
    y_debounce_flag = true;
  }
}

// TOP limit switch
void LimitTop() {
  if (state == HOME) {
    // Should never happen in homing
    reportError("Top limit switch triggered during homing!");
    newState(FAULT);
    return;
  }

  if (!y_debounce_flag) {
    reportError("Top limit switch triggered.");
    newState(FAULT);
    TCNT5 = 0;
    hit_top = true;
    y_debounce_flag = true;
  }
}

// ================== encoder ISRs =====================
// ===== ENCODER QUADRATURE (1x on A rising, read B for direction) =====
void EncoderISR_L() {
  if (digitalRead(encoderB_L)) encoderCountL--;
  else                         encoderCountL++;
}
void EncoderISR_R() {
  if (digitalRead(encoderB_R)) encoderCountR--;
  else                         encoderCountR++;
}


ISR(TIMER1_COMPA_vect) { x_debounce_flag = false; }

// --- CHANGED: use Timer5 compare ISR instead of Timer3
ISR(TIMER5_COMPA_vect) { y_debounce_flag = false; }


void getCommand(){
  String commandArdString = Serial.readStringUntil('\n');
  commandArdString.toCharArray((char *)commandStr, INPUT_BUFFER_SIZE);
  
  char *semicolon = strchr((char*)commandStr, ';');
  if(semicolon != NULL) {
    *semicolon = '\0';
  }
}

void getG01Values(){
  // Extracts X, Y and F components of G01/G1 command and sets Xdist,Ydist and Speed to them. 
  char *ptr = (char *)commandStr;
  float x = 0, y = 0, f = 0;
  
  char *xPtr = strchr(ptr, 'X');
  if (xPtr) {
    x = atof(xPtr + 1);
    Xdist = (int)x; // mm
  }
  char *yPtr = strchr(ptr, 'Y');
  if (yPtr) {
    y = atof(yPtr + 1);
    Ydist = (int)y; // mm
  }
  char *fPtr = strchr(ptr, 'F'); 
  if (fPtr) {
    if(atof(fPtr + 1) != 0){ //If speed is zero or cannot be found, keep existing value
      f = atof(fPtr + 1);
      desiredSpeed = (int)f; // mm/min
    }
  }
  // Print for test
  Serial.print("X: "); Serial.println(Xdist);
  Serial.print("Y: "); Serial.println(Ydist);
  Serial.print("F: "); Serial.println(desiredSpeed);
}


float EncoderCountsToDist(uint16_t encoder_counts ){
  // this function converts encoder counts to a distance in mm
  float shaft_diameter = 12; // shaft diameter in mm
  int cpr_before_gear_box = 12; // counts per revolution
  int gear_ratio = 172;
  int cpr_after_gear_box =  cpr_before_gear_box*gear_ratio;
  float counts_to_rad_ratio = 2*M_PI/cpr_after_gear_box;
  float mm_per_count = 0.5*counts_to_rad_ratio*shaft_diameter;
  float distance = encoder_counts * mm_per_count;
  return distance;  
}

uint16_t DistToEncoderCounts(float distance){
  // this function converts a distance in mm to encoder counts
  float shaft_diameter = 12; // shaft diameter in mm
  int cpr_before_gear_box = 12; // counts per revolution
  int gear_ratio = 172;
  int cpr_after_gear_box =  cpr_before_gear_box*gear_ratio;
  float counts_to_rad_ratio = 2*M_PI/cpr_after_gear_box;
  float mm_per_count = 0.5*counts_to_rad_ratio*shaft_diameter;
  uint16_t encoder_counts = distance/mm_per_count;
  return encoder_counts;  
}

void Controler(float current_position, float target_position, uint16_t* integral, uint16_t* previous_error) {
  uint16_t current_counts = DistToEncoderCounts(current_position);
  uint16_t target_counts = DistToEncoderCounts(target_position);
  float error = target_counts - current_counts;
  float Kp = 0; // Proportional gain
  float Ki = 0; // Integral gain
  float Kd = 0; // Derivative gain
  *integral += error; // Update integral term
  float derivative = error - *previous_error; // Calculate derivative term
  *previous_error = error; // Update previous error
  float output = Kp * error + Ki * (*integral) + Kd * derivative; // PID output need to add feed forward
}