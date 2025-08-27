#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
// CHECK ALLOW LIBRARIES:
#include <stdlib.h>
#include <string.h>
#include <math.h>

volatile float desiredX = 0;    // enc
volatile float desiredY = 0;       // enc
volatile float desiredA = desiredX + desiredY;    // enc
volatile float desiredB = desiredX - desiredY;    // enc

// === A/B position references from integrating velocity (enc) =======
volatile float ReferenceA = 0.0f;
volatile float ReferenceB = 0.0f;

// Internal integrator state (shared by RT integrator)
static unsigned long _lastMsForRef = 0;
static bool _havePrevV = false;
static float _prevVA = 0.0f, _prevVB = 0.0f;

// Move start time (millis)
static unsigned long moveStartMs = 0;
  
// Make these float literals explicitly (no int intermediate)
float maxSafeAcceleration = 7200000.0f;      // enc/min^2
float maxVelocity        = 20000.0f * 60.0f;  // enc/min
float maxsafePWM         = 178;              // raw PWM

// ======= TUNABLES =======
static const int BASE_PWM   = 180;  // starting drive; set just above static friction
static const int PWM_MIN    = 35;  // keep some torque in the slow motor
static const int PWM_MAX    = 255;
static const int KP_NUM     = 6;    // proportional gain (num/den) for balance
static const int KP_DEN     = 10;   // e.g., 0.6
static const int L_PWM_ADJ  = 0;    // empirical trim
static const int R_PWM_ADJ  = 3;    // empirical trim

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

// ===== Quadrature ISR (A rising, read B for direction) =====
void EncoderISR_L() {
  if (digitalRead(encoderB_L)) encoderCountL--;
  else                         encoderCountL++;
}
void EncoderISR_R() {
  if (digitalRead(encoderB_R)) encoderCountR--;
  else                         encoderCountR++;
}


// ===== Hard limits =====
const int x_limit_mm   = 40;   // physical limit of X axis (D19)
const int y_limit_mm = 25;   // physical limit of Y axis (D2)


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

volatile long desiredSpeed = 800L * 60L;     // enc/min
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

// ====  VELOCITY PROFILE AND CONTROL SETUP ==========================
typedef struct {
  float t_acc;  // min
  float d_acc;  // enc
  float d_crs;  // enc
  float t_crs;  // min
} Vprofile;
Vprofile profile;

int main() {
  init();
  Serial.begin(115200);

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

  //attach encoder isrs
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
    } else if (commandType == MOVE) {
        if (!checkMovementCommand()) {
            reportError("Movement command is invalid or missing elements");
            newState(FAULT);
        } else {
            if (!getG01Values()) {
                // getG01Values() already set FAULT and printed the error.
                // DO NOT advance the state.
                return;
            }
            newState(MOVE);   // only if parsing succeeded
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
  // Placeholder for movement
  Serial.println("Moving");
  // ===== Velocity Profile =====
  Move(0, -2000);  // Move 1000 mm in X direction
  // using trapezoidal velocity profile
  
  newState(IDLE);
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

bool getG01Values(){
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
    //   desiredSpeed = (int)f; // mm/min
      //convert to enc/s
      //desiredSpeed = (long)(f/60.0f * DistToEncoderCounts(1.0f)); // enc/s
    }
  }
  // Print for test
    Serial.print("X: "); Serial.println(desiredX);
    Serial.print("Y: "); Serial.println(desiredY);
    Serial.print("A: "); Serial.println(desiredA);
    Serial.print("B: "); Serial.println(desiredB);
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


// ---------- references state ----------
void ResetReferences(unsigned long now_ms) {
  ReferenceA = 0.0f;
  ReferenceB = 0.0f;
  _lastMsForRef = now_ms;
  _havePrevV = false;
  _prevVA = 0.0f;
  _prevVB = 0.0f;
}

void ResetEncoderCounts() {
  encoderCountL = 0;
  encoderCountR = 0;
}

// Real-time (millis-driven) trapezoidal integration
void UpdateReferenceFromVelocity_RT(unsigned long now_ms,
                                    float vA_enc_per_min,
                                    float vB_enc_per_min) {
  unsigned long dms = now_ms - _lastMsForRef;
  if (dms == 0) return;
  _lastMsForRef = now_ms;

  float dt_min = (float)dms / 60000.0f;

  if (_havePrevV) {
    ReferenceA += 0.5f * (vA_enc_per_min + _prevVA) * dt_min; // 
    ReferenceB += 0.5f * (vB_enc_per_min + _prevVB) * dt_min;
  } else {
    ReferenceA += vA_enc_per_min * dt_min;
    ReferenceB += vB_enc_per_min * dt_min;
    _havePrevV = true;
  }
  _prevVA = vA_enc_per_min;
  _prevVB = vB_enc_per_min;
}



// ---------- motors ----------
void PowerMotor(int PWM_L, int PWM_R, int dir_L, int dir_R) {
  digitalWrite(RM_DIR, dir_R);
  digitalWrite(LM_DIR, dir_L);
  analogWrite(RM_PWM, PWM_R);
  analogWrite(LM_PWM, PWM_L);
}


void ABToXY(float* X, float* Y, float A, float B) {
  *X = (A + B) / 2.0f;
  *Y = (A - B) / 2.0f;
}

void XYToAB(float X, float Y, float* A, float* B) {
  *A = X + Y;
  *B = X - Y;
}







// ---------- move (real-time with millis) ----------
void Move(float X, float Y) {
    //print desired X and Y
    Serial.print("Moving to X: "); Serial.print(X); Serial.print(" Y: "); Serial.println(Y);
    desiredX = X;
    desiredY = Y;

    desiredA = desiredX + desiredY;
    desiredB = desiredX - desiredY;

    //PID parameters
    float Kp_left = 1.1f;   // Proportional gain for left motor
    float Kp_right = 1.2f;  // Proportional gain for right motor
    float Ki_left = 0.0f; // Integral gain for left motor
    float Ki_right = 0.0f;// Integral gain for right motor
    float Kd_left = 0.0f; // Derivative gain for left motor
    float Kd_right = 0.0f;// Derivative gain for right motor
    float _prevErrorA = 0.0f, _prevErrorB = 0.0f;

    //velocity feed forward parameters
    float Kff_left = 0.001f;  // Feedforward gain for left motor
    float Kff_right = 0.001f; // Feedforward gain for right motor
    float lookaheadTime = 0.0017f; // around 100 ms
    



  GenerateVProfile();
  Serial.print("t_acc: "); Serial.println(profile.t_acc);
  Serial.print("d_acc: "); Serial.println(profile.d_acc);
  Serial.print("d_crs: "); Serial.println(profile.d_crs);
  Serial.print("t_crs: "); Serial.println(profile.t_crs);

  const uint16_t CONTROL_PERIOD_MS = 10;   // 100 Hz control cadence
  const int      MIN_PWM           = 35;   // deadband
  const int      POS_TOL           = 25;   // counts tolerance to finish

  float targetVA, targetVB, futureVA, futureVB;
  int   maxPWM = (int)maxsafePWM;

  // Prepare
  ResetEncoderCounts();
  moveStartMs = millis();
  ResetReferences(moveStartMs);

  unsigned long lastCtrl = moveStartMs;
  const float totalTimeMin = profile.t_acc + profile.t_crs + profile.t_acc;

  while (1) {
    unsigned long now = millis();
    if ((now - lastCtrl) < CONTROL_PERIOD_MS) continue;
    lastCtrl = now;

    float t_min = (float)(now - moveStartMs) / 60000.0f;

    // Desired velocity
    GetTargetVelocity(t_min, &targetVA, &targetVB);

    // Update reference positions from velocity
    UpdateReferenceFromVelocity_RT(now, targetVA, targetVB);

    GetTargetVelocity(t_min + lookaheadTime, &futureVA, &futureVB); // lookahead (not used here)
    //delta target velocity for feedforward
    float deltaVA = futureVA - targetVA;
    float deltaVB = futureVB - targetVB;

    // Snapshot encoders atomically
    int16_t encL, encR;
    noInterrupts();
    encL = encoderCountL;
    encR = encoderCountR;
    interrupts();

    // Position errors (enc)
    float errorA = ReferenceA - (float)encL; // find error as difference in reference to encoder count
    float errorB = ReferenceB - (float)encR;

    //acculumating error for integral
    static float accErrorA = 0.0f;
    static float accErrorB = 0.0f;
    //simple integration, no need for actual time
    accErrorA += errorA; //accumulate error over time in minutes
    accErrorB += errorB;


    //control
    float uA =  Kp_left * errorA + Ki_left * accErrorA + Kd_left * (errorA - _prevErrorA) + Kff_left * deltaVA;
    float uB =  Kp_right * errorB + Ki_right * accErrorB + Kd_right * (errorB - _prevErrorB) + Kff_right * deltaVB;

    //print effort values from feed forward param
    //Serial.print(" |FF A/B: "); Serial.print(Kff_left * deltaVA, 3); Serial.print("/"); Serial.print(Kff_right * deltaVB, 3);


    _prevErrorA = errorA;
    _prevErrorB = errorB;

    int dirA = (uA >= 0) ? HIGH : LOW;
    int dirB = (uB >= 0) ? HIGH : LOW;
    // Serial.print("============"); 
    // Serial.print(dirA);
    // Serial.print(" L  :  R "); 
    // Serial.print(dirB);
    // Serial.print("============");

    // ---- effort magnitude (float), apply rounding/deadband/clamps while still float
    float magA = fabsf(uA);
    float magB = fabsf(uB);

    // 0 < mag < 1 -> 1 (remove tiny zero band)
    if (magA > 0.0f && magA < 1.0f) magA = 1.0f;
    if (magB > 0.0f && magB < 1.0f) magB = 1.0f;

    // clamp to max
    if (magA > (float)maxPWM) magA = (float)maxPWM;
    if (magB > (float)maxPWM) magB = (float)maxPWM;

    // apply MIN_PWM only if nonzero (keep true zero off)
    if (magA > 0.0f && magA < (float)MIN_PWM) magA = (float)MIN_PWM;
    if (magB > 0.0f && magB < (float)MIN_PWM) magB = (float)MIN_PWM;

    // finally cast to int once
    int pwmA = (int)magA;
    int pwmB = (int)magB;



    PowerMotor(pwmA, pwmB, dirA, dirB);

    // ---------- Debug prints ----------
    // Serial.print("t(min): "); Serial.print(t_min, 3);
    // Serial.print(" | VA/VB: "); Serial.print(targetVA, 1); Serial.print("/"); Serial.print(targetVB, 1);
    // Serial.print(" | RefA/B: "); Serial.print(ReferenceA, 1); Serial.print("/"); Serial.print(ReferenceB, 1);
    // Serial.print(" | EncL/R: "); Serial.print(encL); Serial.print("/"); Serial.print(encR);
    // Serial.print(" | eA/eB: "); Serial.print(errorA, 1); Serial.print("/"); Serial.print(errorB, 1);
    // Serial.print(" | pwmA/B: "); Serial.print(pwmA); Serial.print("/"); Serial.println(pwmB);
     //Serial.print(t_min, 6);  Serial.print(',');
    

    static bool csvHeader = false;
if (!csvHeader) {
  Serial.println("t_min,RefA,RefB,EncL,EncR,TargetVA,TargetVB,ActualVA,ActualVB");
  csvHeader = true;
}

// ---- actual velocity calc (enc/min) ----
static int16_t lastEncL = 0, lastEncR = 0;
static unsigned long lastTime = 0;

float actualVA = 0.0f, actualVB = 0.0f;
if (lastTime != 0) {
  unsigned long dms = now - lastTime;           // ms
  if (dms > 0) {
    float dt_min = (float)dms / 60000.0f;       // minutes
    int32_t dL = (int32_t)encL - (int32_t)lastEncL;
    int32_t dR = (int32_t)encR - (int32_t)lastEncR;
    actualVA = (float)dL / dt_min;              // enc/min
    actualVB = (float)dR / dt_min;              // enc/min
  }
}
// update last* for next tick
lastTime = now;
lastEncL = encL;
lastEncR = encR;

// ---- CSV line ----
// t_min,RefA,RefB,EncL,EncR,TargetVA,TargetVB,ActualVA,ActualVB
Serial.print(t_min, 6);        Serial.print(',');
Serial.print(ReferenceA, 3);   Serial.print(',');
Serial.print(ReferenceB, 3);   Serial.print(',');
Serial.print(encL);            Serial.print(',');
Serial.print(encR);            Serial.print(',');
Serial.print(targetVA, 3);     Serial.print(',');
Serial.print(targetVB, 3);     Serial.print(',');
Serial.print(actualVA, 3);     Serial.print(',');
Serial.println(actualVB, 3);
   static unsigned long atTargetSince = 0;
    // Exit when profile time is done and near the targets, and have been for a short while, based on PWM effort
    if (t_min >= totalTimeMin) {
      if (fabs(errorA) < POS_TOL && fabs(errorB) < POS_TOL) {
        //static unsigned long atTargetSince = 0;
        if (atTargetSince == 0) atTargetSince = now;
        else if ((now - atTargetSince) >= 500) { // 500 ms at target
          // stop motors
          PowerMotor(0, 0, HIGH, HIGH);
          break;
        }
      } else {
        atTargetSince = 0; // reset if we go out of tolerance
      }
    }
  }
}

void GenerateVProfile() {
  float a = maxSafeAcceleration;
  float v = (float)desiredSpeed;
  float x, y;
  ABToXY(&x, &y, desiredA, desiredB);

  float totalmoveXY = sqrtf(x * x + y * y);
  float d_min = (v * v) / a;  // min dist to accel+decel

  if (totalmoveXY <= 2.0f * d_min) {
    Serial.println("Using triangular profile");
    profile.t_acc = sqrtf(totalmoveXY / a);
    profile.d_acc = totalmoveXY / 2.0f;
    profile.d_crs = 0;
    profile.t_crs = 0;
  } else {
    Serial.println("Using trapezoidal profile");
    profile.t_acc = v / a;
    profile.d_acc = 0.5f * a * profile.t_acc * profile.t_acc;
    profile.d_crs = totalmoveXY - 2.0f * profile.d_acc;
    profile.t_crs = profile.d_crs / v;
  }
}

// elapsedMin = (now - moveStartMs) / 60000.0f
void GetTargetVelocity(float elapsedMin, float* targetVA, float* targetVB) {
  float a = maxSafeAcceleration;      // enc/min^2
  float v = (float)desiredSpeed;      // enc/min
  float t_min = elapsedMin;
  if (t_min < 0.0f) t_min = 0.0f;

  float Vtotal = 0.0f;
  bool triangular = (profile.d_crs == 0.0f);

  if (triangular) {
    if (t_min < profile.t_acc) {
      Vtotal = a * t_min;
    } else {
      float t_dec = t_min - profile.t_acc;
      float Vpeak = a * profile.t_acc;
      Vtotal = Vpeak - a * t_dec;
      if (Vtotal < 0.0f) Vtotal = 0.0f;
    }
  } else {
    if (t_min < profile.t_acc) {
      Vtotal = a * t_min;
    } else if (t_min <= (profile.t_acc + profile.t_crs)) {
      Vtotal = v;
    } else {
      float t_dec = t_min - profile.t_crs - profile.t_acc;
      Vtotal = v - a * t_dec;
      if (Vtotal < 0.0f) Vtotal = 0.0f;
    }
  }

  float totalDist = sqrtf(desiredX * desiredX + desiredY * desiredY);

  

  float Vx = (totalDist > 0.0f) ? (desiredX / totalDist) * Vtotal : 0.0f; 
  float Vy = (totalDist > 0.0f) ? (desiredY / totalDist) * Vtotal : 0.0f;

  XYToAB(Vx, Vy, targetVA, targetVB);
}


