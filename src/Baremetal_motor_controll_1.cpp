#include <avr/io.h>

volatile int8_t top_value = 0;


int main(void) {
    // Set pin 4 (PG5) as output
    DDRG |= (1 << PG5);
    // Set pin 5 (PE3) as output
    DDRE |= (1 << PE3);

    // Set pin 4 HIGH
    PORTG |= (1 << PG5);// Motor forward direction
    // Set pin 5 HIGH

    //timer 2 setup
    TCCR2A |= (1<<COM2A0); //toggle on compare match
    //wgm2 for ctc with top ocra
    
    // Set CTC mode (WGM2[2:0] = 010)
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22);  // Prescaler 64

    OCR2A = 249; // 1 ms interval with 16MHz clock and prescaler 64

    TIMSK2 = (1 << OCIE2A); // Enable compare match A interrupt




    while (1) {
    }

}
    ISR(TIMER2_COMPA_vect) {
    // Called every 1 ms
        top_value += 5;
        if (top_value >= 255) {
             top_value = 0;
    }
        

    
}
