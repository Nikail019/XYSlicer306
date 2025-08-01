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



int main(){
  cli();

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

  // // LIMIT SWITCH SETUP
  // //  TOP, RIGHT, LEFT, BOTTOM == D21, D20, D19, 18
  // DDRD &= ~((1 << PD0 ) | (1 << PD1) | (1 << PD2) | (1 << PD3));
  // //enable interrup on those pins on rising edge
  // EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);
  // EICRA |= 0xFF; //ALL ARE HIGH FOR RISING EDGE 
  
  // LIMIT SWITCH INTERRUPT SETUP
  //  TOP, RIGHT, LEFT, BOTTOM == D21, D20, D19, D18
  attachInterrupt(digitalPinToInterrupt(21), LimitTop , RISING);
  attachInterrupt(digitalPinToInterrupt(20), LimitRight , RISING);
  attachInterrupt(digitalPinToInterrupt(19), LimitLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(18), LimitBottom, RISING);
  
  // ENCODER INTERRUPT SETUP
  // LEFT MOTOR (A1, B1), RIGHT MOTOR (A2, B2)
  attatchInterrupt(digitalPinToInterrupt(2), EncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), EncoderA2, CHANGE);
  
 // Pin Change Interrupts
  //B1 -> D15, PJO, PCINT[10], B2 -> D69/A15, PK7, PCINT[23]
  // B1 uses bit 1, B2 uses bit 2
  PCICR |= (1 << PCIE1) | (1 << PCIE2);
  PCMSK1 |= ( 1 << PCINT1O); 
  PCMSK2 |= ( 1 << PCINT23);
  
  sei();

  while(1){
  
  }
}


ISR(PCINT1_vect){
 // Interrupt for Encoder channel B1 
}

ISR(PCINT2_vect){
  // Interrupt for Encoder channel B2
}

void LimitTop(){
  if (x_debounce_flag == 0){
    TCNT1 = 0;
    //method
    hit_top = true;
    x_debounce_flag = true;
  }
}

void LimitRight(){
  if (x_debounce_flag == 0){
    TCNT1 = 0;
    //method
    hit_right = true;
    x_debounce_flag = true;
  }
}

void LimitLeft(){
  if (y_debounce_flag == 0){
    TCNT3 = 0;
    //method
    hit_left = true;
    y_debounce_flag = true;
  }
}

void LimitBottom(){
  if (y_debounce_flag == 0){
    TCNT3 = 0;
    //method
    y_debounce_flag = true;
  }
}

// //x direction limit switches (Timer 1)
// ISR(INT0_vect){
//   //Reset timer 1
//   if (x_debounce_flag == 0){
//     TCNT1 = 0;
//     //method
//     x_debounce_flag = true;
//   }

// }

// ISR(INT1_vect){
//   if (x_debounce_flag == 0){
//     TCNT1 = 0;
//     //method
//     x_debounce_flag = true;
//   }
// }
// // y direction limit switches (timer 3)
// ISR(INT2_vect){
//     if (y_debounce_flag == 0){
//     TCNT3 = 0;
//     //method
//     x_debounce_flag = true;
//   }
// }

// ISR(INT3_vect){
//   if (y_debounce_flag == 0){
//     TCNT3 = 0;
//     //method
//     y_debounce_flag = true;
//   }

// }

ISR(TIMER1_COMPA_vect){
  x_debounce_flag = false;
}

ISR(TIMER3_COMPA_vect){
  y_debounce_flag = false;
}
