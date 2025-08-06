#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

const int M1 = 4;  // Motor direction pin
const int PWM1 = 5;  // Motor PWM pin

const int M2 = 6;  // Motor direction pin
const int PWM2 = 7;  // Motor PWM pin


volatile bool x_debounce_flag = 0;
volatile bool y_debounce_flag = false;
volatile bool hit_top = false;
volatile bool hit_right = false;
volatile bool hit_left = false;
volatile bool hit_bottom = false;
volatile int E1 = 0; // Left
volatile int E2 = 0; // Right



int main(){
  cli();
  Serial.begin(9600);

  // Setup motor pins
  pinMode(M1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  //timer counter 1 with prescaler of 1024

  TCCR1B |= (1 << CS12) | (1 << CS10); 
  TCCR1B = (1 << WGM12); //ctc mode
  OCR1A = 3125;
  //timer counter 3 with prescaler of 1024
  TCCR3B |= (1 << CS32) | (1 << CS30);
  TCCR3B |= (1 << WGM32); // ctc mode
  OCR3A = 3125;
  
  TIMSK1 |= (1 << OCIE1A); //ISR on Compare Match
  TIMSK3 |= (1 << OCIE3A); 

  
  // LIMIT SWITCH INTERRUPT SETUP
  //  TOP, RIGHT, LEFT, BOTTOM == D21, D20, D19, D18
  attachInterrupt(digitalPinToInterrupt(21), LimitTop , RISING);
  attachInterrupt(digitalPinToInterrupt(20), LimitRight , RISING);
  attachInterrupt(digitalPinToInterrupt(19), LimitLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(18), LimitBottom, RISING);
  
  // ENCODER INTERRUPT SETUP
  // LEFT MOTOR (A1, B1), RIGHT MOTOR (A2, B2)
  attachInterrupt(digitalPinToInterrupt(2), EncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), EncoderA2, CHANGE);
  
 // Pin Change Interrupts
  //B1 -> D14, PJO, PCINT[10], B2 -> D69/A15, PK7, PCINT[23]
  // B1 uses bit 1, B2 uses bit 2
  PCICR |= (1 << PCIE1) | (1 << PCIE2);
  PCMSK1 |= ( 1 << PCINT10); 
  PCMSK2 |= ( 1 << PCINT23);
  
  sei();

  while(1){
    moveMotorL(HIGH, 150, 1000);
    delay(1000);  // Pause before repeating
    moveMotorL(LOW, 150, 1000);
    delay(1000);  // Pause before repeating
    moveMotorR(HIGH, 150, 1000);
    delay(1000);  // Pause before repeating
    moveMotorR(LOW, 150, 1000);
    delay(1000);  // Pause before repeating
  
  }
}


void moveMotorL(bool direction, int speed, int duration_ms) {
  digitalWrite(M1, direction ? HIGH : LOW);  // Set motor direction
  analogWrite(PWM1, speed);                    // Set motor speed
  delay(duration_ms);                        // Run motor
  analogWrite(PWM1, 0);                        // Stop motor
}

void moveMotorR(bool direction, int speed, int duration_ms) {
  digitalWrite(M2, direction ? HIGH : LOW);  // Set motor direction
  analogWrite(PWM2, speed);                    // Set motor speed
  delay(duration_ms);                        // Run motor
  analogWrite(PWM2, 0);                        // Stop motor
}



ISR(PCINT1_vect){
 // Interrupt for Encoder channel B1 
 E1++;
  Serial.write("ENC B1: ");
  Serial.write(E1);
  Serial.write("/n");
 
}

ISR(PCINT2_vect){
  // Interrupt for Encoder channel B2
  E2++;
  Serial.write("ENC B2: ");
  Serial.write(E2);
  Serial.write("/n");
}
 
void EncoderA1() {
  E1++;
  Serial.write("ENC A1: ");
  Serial.write(E1);
  Serial.write("/n");
}

void EncoderA2() {
  E2++;
  Serial.write("ENC A2: ");
  Serial.write(E2);
  Serial.write("/n");
  
}

void LimitTop(){
  
  if (x_debounce_flag == 0){
    Serial.write("TOP \n");
    TCNT1 = 0;
    //method
    // turn off motor
    // set state to fault
    hit_top = true;
    x_debounce_flag = true;
  }
}

void LimitRight(){
  
  if (x_debounce_flag == 0){
    Serial.write("RIGHT \n");
    TCNT1 = 0;
    //method
    // turn off motor
    // set state to fault
    hit_right = true;
    x_debounce_flag = true;
  }
}

void LimitLeft(){
  
  if (y_debounce_flag == 0){
    Serial.write("LEFT \n");
    TCNT3 = 0;
    //method
    // turn off motor
    // set state to fault
    hit_left = true;
    y_debounce_flag = true;
  }
}

void LimitBottom(){
  
  if (y_debounce_flag == 0){
    Serial.write("BOTTOM \n");
    TCNT3 = 0;
    //method
    // turn off motor
    // set state to fault
    hit_bottom = true;
    y_debounce_flag = true;
  }
}


ISR(TIMER1_COMPA_vect){
  x_debounce_flag = false;
}

ISR(TIMER3_COMPA_vect){
  y_debounce_flag = false;
}



// #include <avr/io.h>
// #include <util/delay.h>
// #include <avr/interrupt.h>

// // 172:1 gear ratio = 8256 ticks per full gearbox revolution
// // using rising edge only degrees_per_tick = 360.0 / 2064 = 0.1745°

// #define MOTOR_POWER 25
// #define MOTOR_MAX_VOLTAGE 6 // max voltage is 9, 60% for safety

// volatile uint16_t encoder_ticks = 0;

// void setup() {
//     cli(); // Disable interrupts during setup

//     // // Motor direction pin PB6 (digital 12)
//     // DDRB |= (1 << DDB6);

//     // Motor direction pin PG5 (digital 4)
//     DDRG |= (1 << DDG5);

//     // PWM pin PE5 (digital 3) output
//     DDRE |= (1 << DDE5);

//     // Encoder input pin PD0 (INT2) as input
//     DDRD &= ~(1 << DDD0);

//     // Configure external interrupt INT2 on rising edge
//     EICRA |= (1 << ISC21) | (1 << ISC20);  // ISC21:ISC20 = 11 for rising edge
//     EIMSK |= (1 << INT2);                   // Enable INT2 interrupt

//     // Timer 3 setup for Fast PWM mode 14, TOP = ICR3
//     TCCR3A = (1 << COM3C1) | (1 << WGM31);           // Non-inverting mode on OC3C (PE5), WGM31 = 1
//     TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31); // WGM32+WGM33=14 Fast PWM, prescaler=8

//     ICR3 = 2000;    // TOP for 1kHz PWM frequency (16MHz / (8*2000))
//     OCR3C = (MOTOR_POWER * ICR3) / 100; // Set initial duty cycle based on MOTOR_POWER %

//     sei();  // Enable interrupts
// }

// ISR(INT2_vect) {
//     encoder_ticks++;
//     if (encoder_ticks >= 2064) {
//         // Stop motor PWM by setting duty cycle to 0
//         OCR3C = 0;
//     }
// }

// int main(void) {
//     setup();

//     // Set motor direction pin high or low
//     PORTB |= (1 << PORTB6);

//     while (1) {
//     }
// }