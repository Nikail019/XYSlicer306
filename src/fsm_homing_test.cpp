#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// =================== MOTOR PINOUTS ===================
// LEFT MOTOR
const int LM_DIR   = 4;    // DIR
const int LM_PWM   = 5;    // PWM (Timer3 PWM channel)
const int encoderA_L = 21; // ext interrupt
const int encoderB_L = 14; // digital

volatile int16_t encoderCountL = 0;

// RIGHT MOTOR
const int RM_DIR   = 7;    // DIR
const int RM_PWM   = 6;    // PWM (Timer4 PWM channel)
const int encoderA_R = 20; // ext interrupt
const int encoderB_R = 15; // digital

volatile int16_t encoderCountR = 0;

// =================== SWITCH PINS (ACTIVE-LOW) ===================
const int SW_TOP_PIN    = 2;   // INT4
const int SW_RIGHT_PIN  = 3;   // INT5
const int SW_LEFT_PIN   = 19;  // INT2
const int SW_BOTTOM_PIN = 18;  // INT3

static inline bool pressedTop()    { return digitalRead(SW_TOP_PIN)    == LOW; }
static inline bool pressedRight()  { return digitalRead(SW_RIGHT_PIN)  == LOW; }
static inline bool pressedLeft()   { return digitalRead(SW_LEFT_PIN)   == LOW; }
static inline bool pressedBottom() { return digitalRead(SW_BOTTOM_PIN) == LOW; }

// =================== FSM / GCODE ===================
volatile int state;

// STATE DEFINITIONS
#define IDLE  0
#define PARSE 1
#define MOVE  2
#define HOME  3
#define FAULT 4

// GCODE
#define M999     5
#define INVALID  6
#define INPUT_BUFFER_SIZE 64
volatile char commandStr[INPUT_BUFFER_SIZE];

volatile int desiredSpeed = 0; // mm/min
volatile int Xdist; // mm
volatile int Ydist; // mm

// =================== LIMIT SWITCH + DEBOUNCE ===================
volatile bool x_debounce_flag = false;  // RIGHT/LEFT on Timer1
volatile bool y_debounce_flag = false;  // TOP/BOTTOM on Timer5
volatile bool hit_top    = false;
volatile bool hit_right  = false;
volatile bool hit_left   = false;
volatile bool hit_bottom = false;

// =================== ENCODER ISRs ===================
void EncoderISR_L() {
  if (digitalRead(encoderB_L)) encoderCountL++;
  else                         encoderCountL--;
}

// FIX: remove erroneous '!' from original
void EncoderISR_R() {
  if (digitalRead(encoderB_R)) encoderCountR++;
  else                         encoderCountR--;
}

// =================== MOTOR HELPERS ===================
void MoveLeft(bool dir, int speed) {
  digitalWrite(LM_DIR, dir);
  analogWrite(LM_PWM, speed);
  for (int i = 0; i < 10; i++) {
    Serial.print("Encoder Count L: ");
    Serial.println(encoderCountL);
    delay(100);
  }
  analogWrite(LM_PWM, 0);
}

void MoveRight(bool dir, int speed) {
  digitalWrite(RM_DIR, dir);
  analogWrite(RM_PWM, speed);
  for (int i = 0; i < 10; i++) {
    Serial.print("Encoder Count R: ");
    Serial.println(encoderCountR);
    delay(100);
  }
  analogWrite(RM_PWM, 0);
}

void MoveLeftAbsoluteEncoder(int16_t targetCount, int speed) {
  int stopCount = 0;
  if (targetCount > 0) {
    stopCount = targetCount + encoderCountL;
    digitalWrite(LM_DIR, HIGH); // cw
    analogWrite(LM_PWM, speed);
    while (encoderCountL < stopCount) {}
    analogWrite(LM_PWM, 0);
  } else if (targetCount < 0) {
    stopCount = encoderCountL + targetCount;
    digitalWrite(LM_DIR, LOW); // ccw
    analogWrite(LM_PWM, speed);
    while (encoderCountL > stopCount) {}
    analogWrite(LM_PWM, 0);
  }
  analogWrite(LM_PWM, 0);
}

void MoveRightAbsoluteEncoder(int16_t targetCount, int speed) {
  int stopCount = 0;
  if (targetCount > 0) {
    stopCount = targetCount + encoderCountR;
    digitalWrite(RM_DIR, HIGH); // cw
    analogWrite(RM_PWM, speed);
    while (encoderCountR < stopCount) {}
    analogWrite(RM_PWM, 0);
  } else if (targetCount < 0) {
    stopCount = encoderCountR + targetCount;
    digitalWrite(RM_DIR, LOW); // ccw
    analogWrite(RM_PWM, speed);
    while (encoderCountR > stopCount) {}
    analogWrite(RM_PWM, 0);
  }
  analogWrite(RM_PWM, 0);
}

// =================== LIMIT SWITCH ISRs (LEVEL-TRACKING + DEBOUNCE) ===================
// Set flags to current level; only fault if not in HOME.
void LimitTop() {
  if (!y_debounce_flag) {
    y_debounce_flag = true;
    TCNT5 = 0;
    hit_top = pressedTop();
    if (state != HOME && hit_top) {
      Serial.println("Top limit switch triggered unexpectedly.");
      state = FAULT;
    }
  }
}
void LimitRight() {
  if (!x_debounce_flag) {
    x_debounce_flag = true;
    TCNT1 = 0;
    hit_right = pressedRight();
    if (state != HOME && hit_right) {
      Serial.println("Right limit switch triggered unexpectedly.");
      state = FAULT;
    }
  }
}
void LimitLeft() {
  if (!x_debounce_flag) {
    x_debounce_flag = true;
    TCNT1 = 0;
    hit_left = pressedLeft();
    if (state != HOME && hit_left) {
      Serial.println("Left limit switch triggered unexpectedly.");
      state = FAULT;
    }
  }
}
void LimitBottom() {
  if (!y_debounce_flag) {
    y_debounce_flag = true;
    TCNT5 = 0;
    hit_bottom = pressedBottom();
    if (state != HOME && hit_bottom) {
      Serial.println("Bottom limit switch triggered unexpectedly.");
      state = FAULT;
    }
  }
}

// Debounce timer ISRs
ISR(TIMER1_COMPA_vect) { x_debounce_flag = false; }  // RIGHT/LEFT
ISR(TIMER5_COMPA_vect) { y_debounce_flag = false; }  // TOP/BOTTOM

// =================== HOMING (NON-BLOCKING) ===================
// Sub-states for homing
enum HomingPhase : uint8_t {
  H_IDLE = 0,
  H_START,
  H_MOVE_LEFT,     // drive left until LEFT pressed
  H_STOP_LEFT,     // stop motors after left hit
  H_MOVE_DOWN,     // drive down until BOTTOM pressed
  H_STOP_DOWN,     // stop motors after bottom hit
  H_DONE
};
volatile HomingPhase hphase = H_IDLE;
unsigned long h_tStart = 0; // for optional timeouts

// parameters
const uint8_t PWM_LEFT_APPROACH  = 200; // first leg
const uint8_t PWM_DOWN_R         = 150;
const uint8_t PWM_DOWN_L         = 155;
const unsigned long HOMING_TIMEOUT_MS = 15000;

// forward decl
void newState(int ns);
bool checkForCommand();
void parseCommand();
void parseErrorCommand();
void simulateMovement();
void reportError(const char *errorCode);
void reportState();
void getCommand();
void getG01Values();
float EncoderCountsToDist(uint16_t encoder_counts);
uint16_t DistToEncoderCounts(float distance);
void Controler(float current_position, float target_position, uint16_t* integral, uint16_t* previous_error);

// start homing (called when entering HOME)
void homingStart() {
  // clear/seed flags to current levels
  hit_left   = pressedLeft();
  hit_bottom = pressedBottom();

  hphase = H_START;
  h_tStart = millis();
  Serial.println("Homing: start");
}

// call repeatedly while state == HOME
void homingUpdate() {
  switch (hphase) {
    case H_START: {
      // begin moving left: both motors CCW (DIR LOW)
      digitalWrite(RM_DIR, LOW);
      digitalWrite(LM_DIR, LOW);
      analogWrite(RM_PWM, PWM_LEFT_APPROACH);
      analogWrite(LM_PWM, PWM_LEFT_APPROACH);
      Serial.println("Homing: moving LEFT to hit LEFT switch...");
      hphase = H_MOVE_LEFT;
      h_tStart = millis();
    } break;

    case H_MOVE_LEFT: {
      if (hit_left) {
        hphase = H_STOP_LEFT;
      } else if (millis() - h_tStart > HOMING_TIMEOUT_MS) {
        Serial.println("Homing ERROR: timeout waiting for LEFT.");
        analogWrite(RM_PWM, 0);
        analogWrite(LM_PWM, 0);
        newState(FAULT);
      }
    } break;

    case H_STOP_LEFT: {
      analogWrite(RM_PWM, 0);
      analogWrite(LM_PWM, 0);
      Serial.println("Homing: LEFT hit, motors stopped.");
      // small settle without blocking: just proceed next tick
      hphase = H_MOVE_DOWN;
    } break;

    case H_MOVE_DOWN: {
      // Move down: Left CW, Right CCW (DIR: L=LOW/HIGH? Using working standalone: RM=HIGH, LM=LOW)
      digitalWrite(RM_DIR, HIGH); // right CCW
      digitalWrite(LM_DIR, LOW);  // left  CW
      analogWrite(RM_PWM, PWM_DOWN_R);
      analogWrite(LM_PWM, PWM_DOWN_L);
      Serial.println("Homing: moving DOWN to hit BOTTOM switch...");
      h_tStart = millis();
      hphase = H_STOP_DOWN; // next phase waits for condition while motors running
    } break;

    case H_STOP_DOWN: {
      if (hit_bottom) {
        analogWrite(RM_PWM, 0);
        analogWrite(LM_PWM, 0);
        Serial.println("Homing: BOTTOM hit, motors stopped.");
        // zero encoders
        encoderCountL = 0;
        encoderCountR = 0;
        hphase = H_DONE;
      } else if (millis() - h_tStart > HOMING_TIMEOUT_MS) {
        Serial.println("Homing ERROR: timeout waiting for BOTTOM.");
        analogWrite(RM_PWM, 0);
        analogWrite(LM_PWM, 0);
        newState(FAULT);
      }
    } break;

    case H_DONE: {
      Serial.println("Homing: complete.");
      hphase = H_IDLE;
      newState(IDLE);
    } break;

    case H_IDLE:
    default:
      // nothing
      break;
  }
}

// =================== MAIN ===================
int main() {
  init();
  Serial.begin(9600);

  cli();
  // === Debounce timers ===
  // Timer1 -> X (RIGHT/LEFT)
  TCCR1A = 0; TCCR1B = 0;
  TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTC, /1024
  OCR1A = 3125;                                       // ~200 ms
  TIMSK1 |= (1 << OCIE1A);

  // Timer5 -> Y (TOP/BOTTOM)  (moved off Timer3 to avoid PWM conflict with pin 5)
  TCCR5A = 0; TCCR5B = 0;
  TCCR5B |= (1 << WGM52) | (1 << CS52) | (1 << CS50); // CTC, /1024
  OCR5A = 3125;                                       // ~200 ms
  TIMSK5 |= (1 << OCIE5A);
  sei();

  // Motor pins
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);
  analogWrite(LM_PWM, 0);
  analogWrite(RM_PWM, 0);

  // Switch pins (active-LOW pullups)
  pinMode(SW_TOP_PIN,    INPUT_PULLUP);
  pinMode(SW_RIGHT_PIN,  INPUT_PULLUP);
  pinMode(SW_LEFT_PIN,   INPUT_PULLUP);
  pinMode(SW_BOTTOM_PIN, INPUT_PULLUP);

  // Initialize switch flags to current levels
  hit_top    = pressedTop();
  hit_right  = pressedRight();
  hit_left   = pressedLeft();
  hit_bottom = pressedBottom();

  // Switch interrupts on CHANGE (level tracking)
  attachInterrupt(digitalPinToInterrupt(SW_TOP_PIN),    LimitTop,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_RIGHT_PIN),  LimitRight,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_LEFT_PIN),   LimitLeft,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_BOTTOM_PIN), LimitBottom, CHANGE);

  // Encoders
  pinMode(encoderA_L, INPUT);
  pinMode(encoderB_L, INPUT);
  pinMode(encoderA_R, INPUT);
  pinMode(encoderB_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA_L), EncoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_R), EncoderISR_R, RISING);

  newState(IDLE);

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
        // non-blocking: drive homing sub-state machine
        if (hphase == H_IDLE) homingStart();
        homingUpdate();
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

// =================== FSM helpers ===================
void newState(int ns) {
  state = ns;
  reportState();
}

void parseCommand() {
  getCommand();
  int commandType = INVALID;
  if (strstr(commandStr, "M999") || strstr(commandStr, "m999")) {
    commandType = M999;
  } else if (strstr(commandStr, "G01") || strstr(commandStr, "g01") ||
             strstr(commandStr, "g1")  || strstr(commandStr, "G1")) {
    commandType = MOVE;
  } else if (strstr(commandStr, "G28") || strstr(commandStr, "g28")) {
    commandType = HOME;
  } else {
    commandType = INVALID;
  }

  if (commandType == HOME) {
    newState(HOME);

  } else if (commandType == INVALID || commandType == M999) {
    reportError("Command is unrecognised or invalid");
    newState(FAULT);

  } else if (commandType == MOVE) {
    if (!((strstr(commandStr, "X") && strstr(commandStr, "Y")) &&
          (desiredSpeed != 0 || strstr(commandStr, "F")))) {
      reportError("Movement command is invalid or missing elements");
      newState(FAULT);
    } else {
      getG01Values();
      newState(MOVE);
    }
  }
}

void parseErrorCommand() {
  getCommand();
  int commandType = INVALID;
  if (strstr(commandStr, "M999") || strstr(commandStr, "m999")) commandType = M999;
  else if (strstr(commandStr, "G01") || strstr(commandStr, "g01") ||
           strstr(commandStr, "g1")  || strstr(commandStr, "G1")) commandType = MOVE;
  else if (strstr(commandStr, "G28") || strstr(commandStr, "g28")) commandType = HOME;
  else commandType = INVALID;

  if (commandType == M999) {
    newState(IDLE);
  } else if (commandType == INVALID) {
    reportError("Command is unrecognised");
  } else if (commandType == MOVE || commandType == HOME) {
    reportError("This type of command is invalid in the error state");
  }
}

bool checkForCommand() {
  return Serial.available() > 0;
}

void reportError(const char *errorCode) { Serial.println(errorCode); }

void reportState() {
  Serial.print("State is: ");
  Serial.println(state);
}

// =================== Movement (unchanged) ===================
void simulateMovement() {
  Serial.println("Starting movement...");

  int16_t deltaA = DistToEncoderCounts(Xdist + Ydist); // Left motor
  int16_t deltaB = DistToEncoderCounts(Xdist - Ydist); // Right motor
  //feed rate calculation
  const int MAX_SPEED_MM_PER_MIN = 2000;
  const int MAX_PWM = 255;
  int pwmSpeed = (desiredSpeed * MAX_PWM) / MAX_SPEED_MM_PER_MIN;
  if (pwmSpeed > 255) pwmSpeed = 255;
  if (pwmSpeed < 50)  pwmSpeed = 50;

  Serial.print("Target counts - Left: "); Serial.print(deltaA);
  Serial.print(" Right: "); Serial.println(deltaB);

  MoveLeftAbsoluteEncoder(deltaA, pwmSpeed);
  MoveRightAbsoluteEncoder(deltaB, pwmSpeed);

  Serial.println("Movement complete");
  newState(IDLE);
}

// =================== Command parsing utils ===================
void getCommand(){
  String commandArdString = Serial.readStringUntil('\n');
  commandArdString.toCharArray((char *)commandStr, INPUT_BUFFER_SIZE);
  char *semicolon = strchr((char*)commandStr, ';');
  if(semicolon != NULL) *semicolon = '\0';
}

void getG01Values(){
  char *ptr = (char *)commandStr;
  float x = 0, y = 0, f = 0;

  char *xPtr = strchr(ptr, 'X');
  if (xPtr) { x = atof(xPtr + 1); Xdist = (int)x; }
  char *yPtr = strchr(ptr, 'Y');
  if (yPtr) { y = atof(yPtr + 1); Ydist = (int)y; }
  char *fPtr = strchr(ptr, 'F');
  if (fPtr) {
    float fv = atof(fPtr + 1);
    if (fv != 0) desiredSpeed = (int)fv;
  }
  Serial.print("X: "); Serial.println(Xdist);
  Serial.print("Y: "); Serial.println(Ydist);
  Serial.print("F: "); Serial.println(desiredSpeed);
}

// =================== Kinematics helpers ===================
float EncoderCountsToDist(uint16_t encoder_counts ){
  float shaft_diameter = 12; // mm
  int cpr_before_gear_box = 12;
  int gear_ratio = 172;
  int cpr_after_gear_box =  cpr_before_gear_box*gear_ratio;
  float counts_to_rad_ratio = 2*M_PI/cpr_after_gear_box;
  float mm_per_count = 0.5*counts_to_rad_ratio*shaft_diameter;
  float distance = encoder_counts * mm_per_count;
  return distance;
}

uint16_t DistToEncoderCounts(float distance){
  float shaft_diameter = 12; // mm
  int cpr_before_gear_box = 12;
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
  float Kp = 0, Ki = 0, Kd = 0;
  *integral += error;
  float derivative = error - *previous_error;
  *previous_error = error;
  float output = Kp * error + Ki * (*integral) + Kd * derivative;
  (void)output;
}