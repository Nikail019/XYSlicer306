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

  // ==============================(MOTOR ISRS AND FUNCTIONS)===========================

  // === ENCODER ISRs ===
  void EncoderISR_L() {
    if (digitalRead(encoderB_L)) encoderCountL++;
    else                         encoderCountL--;
  }

  void EncoderISR_R() {
    if (digitalRead(!encoderB_R)) encoderCountR++; //invert the encoder reading to correspond CW as increasing encoder counts
    else                         encoderCountR--;
  }

  void MoveLeft(bool dir, int speed) {
    digitalWrite(LM_DIR, dir);
    analogWrite(LM_PWM, speed);
    for (int i = 0; i < 10; i++) {
      Serial.print("Encoder Count L: ");
      Serial.println(encoderCountL);
      delay(100); // Move for 1 second
    }
    analogWrite(LM_PWM, 0); // Stop
  }

  void MoveRight(bool dir, int speed) {
    digitalWrite(RM_DIR, dir);
    analogWrite(RM_PWM, speed);
    for (int i = 0; i < 10; i++) {
      Serial.print("Encoder Count R: ");
      Serial.println(encoderCountR);
      delay(100); // Move for 1 second
    }
    analogWrite(RM_PWM, 0); // Stop
  }

  void MoveLeftAbsoluteEncoder(int16_t targetCount, int speed) {
    int stopCount = 0;

    if (targetCount > 0){
      //Current Left encoder count plus target is our new stopping count
      stopCount = targetCount + encoderCountL;

      //intiate motor movement at our speed
      analogWrite(LM_DIR, true); //cw
      analogWrite(LM_PWM, speed);
      while (encoderCountL < stopCount){
        //wait until we reach or exceed desired encoder count
      }
      analogWrite(LM_PWM, 0);
    }

    if (targetCount < 0){
      //Current Left encoder count minus target is our new stopping count
      stopCount = encoderCountL - targetCount;

      //intiate motor movement at our speed
      analogWrite(LM_DIR, false); //ccw
      analogWrite(LM_PWM, speed);
      while (encoderCountL > stopCount){
        //wait until we reach or exceed desired encoder count
      }
      analogWrite(LM_PWM, 0);
    }
    analogWrite(LM_PWM, 0); // Stop
  }

  void MoveRightAbsoluteEncoder(int16_t targetCount, int speed) {
    int stopCount = 0;

    if (targetCount > 0){
      //Current Right encoder count plus target is our new stopping count
      stopCount = targetCount + encoderCountR;

      //intiate motor movement at our speed
      analogWrite(RM_DIR, true); //cw
      analogWrite(RM_PWM, speed);
      while (encoderCountR < stopCount){
        //wait until we reach or exceed desired encoder count
      }
      analogWrite(RM_PWM, 0);
    }

    if (targetCount < 0){
      //Current Right encoder count minus target is our new stopping count
      stopCount = encoderCountR - targetCount;

      //intiate motor movement at our speed
      analogWrite(RM_DIR, false); //ccw
      analogWrite(RM_PWM, speed);
      while (encoderCountR > stopCount){
        //wait until we reach or exceed desired encoder count
      }
      analogWrite(RM_PWM, 0);
    }
    analogWrite(RM_PWM, 0); // Stop
  }


  // ===============================================================

  int main() {
    init();
    Serial.begin(9600);

    cli();
    /*TIMER SETUP*/
    // timer counter 1 with prescaler of 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);
    TCCR1B |= (1 << WGM12);  // ctc mode
    OCR1A = 3125;
    // timer counter 3 with prescaler of 1024
    TCCR3B |= (1 << CS32) | (1 << CS30);
    TCCR3B |= (1 << WGM32);  // ctc mode
    OCR3A = 3125;

    TIMSK1 |= (1 << OCIE1A);  // ISR on Compare Match
    TIMSK3 |= (1 << OCIE3A);

    /*LIMIT SWITCH SETUP*/
    //  TOP, RIGHT, LEFT, BOTTOM == D2, D3, D19, D18
    attachInterrupt(digitalPinToInterrupt(2), LimitTop, RISING);
    attachInterrupt(digitalPinToInterrupt(3), LimitRight, RISING);
    attachInterrupt(digitalPinToInterrupt(19), LimitLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(18), LimitBottom, RISING);

    //Motor Initilisation
    pinMode(LM_DIR, OUTPUT);
    pinMode(LM_PWM, OUTPUT);
    pinMode(RM_DIR, OUTPUT);
    pinMode(RM_PWM, OUTPUT);

    // Encoder pins
    pinMode(encoderA_L, INPUT);
    pinMode(encoderB_L, INPUT);
    pinMode(encoderA_R, INPUT);
    pinMode(encoderB_R, INPUT);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(encoderA_L), EncoderISR_L, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderA_R), EncoderISR_R, RISING);
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
          digitalWrite(LM_PWM, LOW);
          digitalWrite(RM_PWM, LOW);
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
    // Placeholder for homing procedure
    Serial.println("Homing BRuv");

    //homing function
    // move horizontal left till left switch hit
    // both motors counter clockwise

    //thingy will move left
    digitalWrite(RM_DIR, false);
    digitalWrite(LM_DIR, false); 

    Serial.println("Moving left to hit left limit switch...");

    analogWrite(RM_PWM, 100);
    analogWrite(LM_PWM, 100);

    Serial.println("Waiting for left limit switch to be hit...");

    while(!hit_left){

    }
    analogWrite(RM_PWM, 0);
    analogWrite(LM_PWM, 0);
    Serial.println("Left limit switch hit, stopping motors.");

    //move down till bottom switch hit (Left Motor Clockwise Right Motor Counter Clockwise)
    digitalWrite(RM_DIR, false);
    digitalWrite(LM_DIR, true);
    analogWrite(RM_PWM, 100);
    analogWrite(LM_PWM, 100);
    while(!hit_bottom){

    } 
    analogWrite(RM_PWM, 0);
    analogWrite(LM_PWM, 0);

    //reset encoder counts
    encoderCountL = 0;
    encoderCountR = 0;
    newState(IDLE);

  }

  void simulateMovement() {
      Serial.println("Starting movement...");

      // --- Convert distances to encoder counts for CoreXY ---
      // CoreXY kinematics: deltaA = deltaX + deltaY, deltaB = deltaX - deltaY
      int16_t deltaA = DistToEncoderCounts(Xdist + Ydist); // Left motor
      int16_t deltaB = DistToEncoderCounts(Xdist - Ydist); // Right motor

      // --- Convert desiredSpeed (mm/min) to PWM value ---
      const int MAX_SPEED_MM_PER_MIN = 2000; // your max mm/min
      const int MAX_PWM = 255;
      int pwmSpeed = (desiredSpeed * MAX_PWM) / MAX_SPEED_MM_PER_MIN;
      if (pwmSpeed > 255) pwmSpeed = 255;
      if (pwmSpeed < 50) pwmSpeed = 50; // avoid too slow

      // --- Move motors to target counts ---
      Serial.print("Target counts - Left: "); Serial.print(deltaA);
      Serial.print(" Right: "); Serial.println(deltaB);

      MoveLeftAbsoluteEncoder(deltaA, pwmSpeed);
      MoveRightAbsoluteEncoder(deltaB, pwmSpeed);

      Serial.println("Movement complete");
      newState(IDLE);
  }


  void reportError(const char *errorCode) { Serial.println(errorCode); }

  void reportState() {
    Serial.print("State is: ");
    Serial.println(state);
  }

  void LimitTop() {
    if (y_debounce_flag == 0) {
      reportError("Top limit switch triggered unexpectedly.");
      newState(FAULT);
      TCNT3 = 0;
      hit_top = true;
      y_debounce_flag = true;
    }
  }

  void LimitRight() {
    if (x_debounce_flag == 0) {
      reportError("Right limit switch triggered unexpectedly.");
      newState(FAULT);
      TCNT1 = 0;
      hit_right = true;
      x_debounce_flag = true;
    }
  }

  void LimitLeft() {
    if (x_debounce_flag == 0) {
      reportError("Left limit switch triggered unexpectedly.");
      newState(FAULT);
      TCNT1 = 0;
      hit_left = true;
      x_debounce_flag = true;
    }
  }

  void LimitBottom() {
    if (y_debounce_flag == 0) {
      reportError("Bottom limit switch triggered unexpectedly.");
      newState(FAULT);
      TCNT3 = 0;
      hit_bottom = true;
      y_debounce_flag = true;
    }
  }

  ISR(TIMER1_COMPA_vect) { x_debounce_flag = false; }

  ISR(TIMER3_COMPA_vect) { y_debounce_flag = false; }


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
    float output = Kp * error + Ki * (*integral) + Kd * derivative; // PID output
  }
