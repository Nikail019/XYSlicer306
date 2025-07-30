#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

bool x_debounce_flag = 0;
bool y_debounce_flag = false;



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


  //setup int0:4 pins for input
  DDRD &= ~((1 << PD0 ) | (1 << PD1) | (1 << PD2) | (1 << PD3));

  //enable interrup on those pins on rising edge
  EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);
  EICRA |= 0xFF; //ALL ARE HIGH FOR RISING EDGE 
  sei();

  while(1){
  
  }
}

//x direction limit switches (Timer 1)
ISR(INT0_vect){
  //Reset timer 1
  if (x_debounce_flag == 0){
    TCNT1 = 0;
    //method
    x_debounce_flag = true;
  }

}

ISR(INT1_vect){
  if (x_debounce_flag == 0){
    TCNT1 = 0;
    //method
    x_debounce_flag = true;
  }
}
// y direction limit switches (timer 3)
ISR(INT2_vect){
    if (y_debounce_flag == 0){
    TCNT3 = 0;
    //method
    x_debounce_flag = true;
  }
}

ISR(INT3_vect){
  if (y_debounce_flag == 0){
    TCNT3 = 0;
    //method
    y_debounce_flag = true;
  }

}

ISR(TIMER1_COMPA_vect){
  x_debounce_flag = false;
}

ISR(TIMER3_COMPA_vect){
  y_debounce_flag = false;
}
