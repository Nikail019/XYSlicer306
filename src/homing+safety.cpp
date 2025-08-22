#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
// CHECK ALLOW LIBRARIES:
#include <stdlib.h>
#include <string.h>
#include <math.h>

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


// ======= TUNABLES =======
static const int BASE_PWM   = 180;  // starting drive; set just above static friction
static const int PWM_MIN    = 60;   // keep some torque in the slow motor
static const int PWM_MAX    = 255;
static const int KP_NUM     = 6;    // proportional gain (num/den) for balance
static const int KP_DEN     = 10;   // e.g., 0.6

inline int clampPWM(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }

inline bool abortIfFault(const char* where){
  if (state == FAULT){
    analogWrite(RM_PWM, 0); analogWrite(LM_PWM, 0);
    Serial.print("Homing aborted due to FAULT ("); Serial.print(where); Serial.println(").");
    return true;
  }
  return false;
}

// Drive with closed-loop balance where both motors spin the SAME direction.
// We equalize the moved counts since the start of this segment.
void driveBalancedSameDir(uint8_t pwmBase, bool dirL, bool dirR,
                          volatile int16_t& encL, volatile int16_t& encR,
                          int16_t startL, int16_t startR)
{
  digitalWrite(LM_DIR, dirL);
  digitalWrite(RM_DIR, dirR);

  // Initial equal PWM
  int pwmL = pwmBase, pwmR = pwmBase;
  analogWrite(LM_PWM, pwmL);
  analogWrite(RM_PWM, pwmR);

  // Balance loop iteration (call this in a while-loop body)
  auto balanceStep = [&](void){
    // moved counts (absolute progress since start, because direction may be LOW/HIGH)
    int32_t movedL = (int32_t)encL - (int32_t)startL; if (movedL < 0) movedL = -movedL;
    int32_t movedR = (int32_t)encR - (int32_t)startR; if (movedR < 0) movedR = -movedR;

    int32_t err = (int32_t)movedL - (int32_t)movedR; // + => L ahead, - => R ahead

    // simple P: push PWM toward the lagging side
    pwmL = clampPWM((int)(pwmBase - (err * KP_NUM) / KP_DEN));
    pwmR = clampPWM((int)(pwmBase + (err * KP_NUM) / KP_DEN));

    // keep some minimum to avoid stalling the weaker motor
    if (pwmL && pwmL < PWM_MIN) pwmL = PWM_MIN;
    if (pwmR && pwmR < PWM_MIN) pwmR = PWM_MIN;

    analogWrite(LM_PWM, pwmL);
    analogWrite(RM_PWM, pwmR);
  };

  // expose inner lambda for use in loops
  while (false) { balanceStep(); } // no-op; just here to show scope
}

// Drive with closed-loop balance where motors spin OPPOSITE directions (CoreXY Y move).
// We equalize |moved counts| (magnitudes) since the start of this segment.
void driveBalancedOppDir(uint8_t pwmBase, bool dirL, bool dirR,
                         volatile int16_t& encL, volatile int16_t& encR,
                         int16_t startL, int16_t startR)
{
  digitalWrite(LM_DIR, dirL);
  digitalWrite(RM_DIR, dirR);

  int pwmL = pwmBase, pwmR = pwmBase;
  analogWrite(LM_PWM, pwmL);
  analogWrite(RM_PWM, pwmR);

  auto balanceStep = [&](void){
    int32_t movedL = (int32_t)encL - (int32_t)startL; if (movedL < 0) movedL = -movedL;
    int32_t movedR = (int32_t)encR - (int32_t)startR; if (movedR < 0) movedR = -movedR;

    int32_t err = (int32_t)movedL - (int32_t)movedR; // + => L ahead, - => R ahead

    int newL = (int)(pwmBase - (err * KP_NUM) / KP_DEN);
    int newR = (int)(pwmBase + (err * KP_NUM) / KP_DEN);
    newL = clampPWM(newL); newR = clampPWM(newR);
    if (newL && newL < PWM_MIN) newL = PWM_MIN;
    if (newR && newR < PWM_MIN) newR = PWM_MIN;

    pwmL = newL; pwmR = newR;
    analogWrite(LM_PWM, pwmL);
    analogWrite(RM_PWM, pwmR);
  };

  while (false) { balanceStep(); }
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
  // We'll keep moved counts equal while waiting for LEFT switch.
  {
    Serial.println("Moving left to hit left limit switch...");
    Serial.println("Balancing counts for pure X-...");

    const bool dirL = LOW; 
    const bool dirR = LOW;

    int16_t startL = encoderCountL;
    int16_t startR = encoderCountR;

    // prime outputs
    digitalWrite(LM_DIR, dirL);
    digitalWrite(RM_DIR, dirR);
    analogWrite(LM_PWM, BASE_PWM);
    analogWrite(RM_PWM, BASE_PWM);

    // Wait until left limit hit or FAULT
    while (!hit_left && state != FAULT) {
      // Balance progress
      int32_t movedL = (int32_t)encoderCountL - (int32_t)startL; if (movedL < 0) movedL = -movedL;
      int32_t movedR = (int32_t)encoderCountR - (int32_t)startR; if (movedR < 0) movedR = -movedR;
      int32_t err = movedL - movedR;

      int pwmL = BASE_PWM - (int)((err * KP_NUM) / KP_DEN);
      int pwmR = BASE_PWM + (int)((err * KP_NUM) / KP_DEN);
      pwmL = clampPWM(pwmL); pwmR = clampPWM(pwmR);
      if (pwmL && pwmL < PWM_MIN) pwmL = PWM_MIN;
      if (pwmR && pwmR < PWM_MIN) pwmR = PWM_MIN;

      analogWrite(LM_PWM, pwmL);
      analogWrite(RM_PWM, pwmR);
      delay(5);
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
  // Keep abs moved counts equal while waiting for bottom switch.
  {
    Serial.println("Moving down to hit bottom limit switch...");
    const bool dirL = LOW;  
    const bool dirR = HIGH;

    int16_t startL = encoderCountL;
    int16_t startR = encoderCountR;

    digitalWrite(LM_DIR, dirL);
    digitalWrite(RM_DIR, dirR);
    analogWrite(LM_PWM, BASE_PWM);
    analogWrite(RM_PWM, BASE_PWM);

    while (!hit_bottom && state != FAULT) {
      int32_t movedL = (int32_t)encoderCountL - (int32_t)startL; if (movedL < 0) movedL = -movedL;
      int32_t movedR = (int32_t)encoderCountR - (int32_t)startR; if (movedR < 0) movedR = -movedR;
      int32_t err = movedL - movedR;

      int pwmL = BASE_PWM - (int)((err * KP_NUM) / KP_DEN);
      int pwmR = BASE_PWM + (int)((err * KP_NUM) / KP_DEN);
      pwmL = clampPWM(pwmL); pwmR = clampPWM(pwmR);
      if (pwmL && pwmL < PWM_MIN) pwmL = PWM_MIN;
      if (pwmR && pwmR < PWM_MIN) pwmR = PWM_MIN;

      analogWrite(LM_PWM, pwmL);
      analogWrite(RM_PWM, pwmR);

      delay(5);
    }

    if (abortIfFault("during bottom-seek")) return;

    analogWrite(LM_PWM, 0);
    analogWrite(RM_PWM, 0);
    Serial.println("Bottom limit switch hit, stopping motors.");
  }

  //back off and re-home slowly using same loops.
  // move motor 



  // Zero encoders at home
  encoderCountL = 0;
  encoderCountR = 0;

  Serial.println("Homing complete.");
  newState(IDLE);
}


void simulateMovement() {
  // Placeholder for movement
  Serial.println("Moving");
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


