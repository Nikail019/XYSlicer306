#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h> // for TWCR/TWEN

// === PINOUTS ===
// Left motor + encoder
const int LM_DIR     = 4;
const int LM_PWM     = 5;
const int encoderA_L = 21;  // interrupt pin 
const int encoderB_L = 14;  // digital pin

// Right motor + encoder (swapped)
const int RM_DIR     = 7;   // direction pin
const int RM_PWM     = 6;   // PWM pin
const int encoderA_R = 20;  // interrupt pin (SDA)
const int encoderB_R = 15;  // digital pin

// Encoder counts
volatile int16_t encoderCountL = 0;
volatile int16_t encoderCountR = 0;


// === ENCODER ISRs ===
void EncoderISR_L() {
  if (digitalRead(encoderB_L)) encoderCountL++;
  else                         encoderCountL--;
}

void EncoderISR_R() {
  if (digitalRead(encoderB_R)) encoderCountR++; //invert the encoder reading to correspond CW as increasing encoder counts
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
  Serial.print("Target Count: ");
  Serial.println(targetCount);

  if (targetCount > 0){
    //Current Left encoder count plus target is our new stopping count
    stopCount = targetCount + encoderCountL;

    //intiate motor movement at our speed
    digitalWrite(LM_DIR, false); //cw
    analogWrite(LM_PWM, speed);
    while (encoderCountL < stopCount){
      //wait until we reach or exceed desired encoder count
    }
    analogWrite(LM_PWM, 0);
  }

  Serial.print("Target Count: ");
  Serial.println(targetCount);
  if (targetCount < 0){
    //Current Left encoder count minus target is our new stopping count
    stopCount = encoderCountL - targetCount;
    Serial.print("Lenc Count: ");
    Serial.println(encoderCountL);
    Serial.print("Stop Count: ");
    Serial.println(stopCount);
    //intiate motor movement at our speed
    digitalWrite(LM_DIR, true); //ccw
    analogWrite(LM_PWM, speed);
    while (encoderCountL > stopCount){
      //wait until we reach or exceed desired encoder count
      Serial.print("Encoder Count L: ");
      Serial.println(encoderCountL);
      delay(10); // Allow time for encoder to update
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
    digitalWrite(RM_DIR, false); //cw
    analogWrite(RM_PWM, speed);
    while (encoderCountR < stopCount){
      //wait until we reach or exceed desired encoder count
    }
    analogWrite(RM_PWM, 0);
  }

  if (targetCount < 0){
    //Current Right encoder count minus target is our new stopping count
    stopCount = encoderCountR - targetCount;
    Serial.print("Stop Count: ");
    Serial.println(stopCount);

    //intiate motor movement at our speed
    digitalWrite(RM_DIR, true); //ccw
    analogWrite(RM_PWM, speed);
    while (encoderCountR > stopCount){
      Serial.print("Encoder Count R: ");
      Serial.println(encoderCountR);
      delay(10); // Allow time for encoder to update
      //wait until we reach or exceed desired encoder count
    }
    analogWrite(RM_PWM, 0);
  }
  analogWrite(RM_PWM, 0); // Stop
}


int main() {
  init();
  Serial.begin(9600);


  // Motor pins
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

  // Reset counts
  encoderCountL = 0;
  encoderCountR = 0;

  // === Example Moves ===
  while (1) {
    encoderCountL = 0;
    encoderCountR = 0;
    // MoveLeftAbsoluteEncoder(1000, 125);
    // delay(500);
    // MoveRightAbsoluteEncoder(1000, 125);
    // delay(500);

    MoveLeftAbsoluteEncoder(-1000, 125);
    delay(500);
    MoveRightAbsoluteEncoder(-1000, 125);
    delay(500);

    // digitalWrite(LM_DIR, LOW); //ccw
    // digitalWrite(RM_DIR, LOW); //ccw
    // analogWrite(LM_PWM, 125);
    // analogWrite(RM_PWM, 125);
    // delay(1000); // Move for 1 second

  }
  return 0;
}