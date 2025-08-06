#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

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
  //Timer Setup

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
  
  }
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
  Serial.write("TOP \n");
  if (x_debounce_flag == 0){
    TCNT1 = 0;
    //method
    // turn off motor
    // set state to fault
    hit_top = true;
    x_debounce_flag = true;
  }
}

void LimitRight(){
  Serial.write("RIGHT \n");
  if (x_debounce_flag == 0){
    TCNT1 = 0;
    //method
    // turn off motor
    // set state to fault
    hit_right = true;
    x_debounce_flag = true;
  }
}

void LimitLeft(){
  Serial.write("LEFT \n");
  if (y_debounce_flag == 0){
    TCNT3 = 0;
    //method
    // turn off motor
    // set state to fault
    hit_left = true;
    y_debounce_flag = true;
  }
}

void LimitBottom(){
  Serial.write("BOTTOM \n");
  if (y_debounce_flag == 0){
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
