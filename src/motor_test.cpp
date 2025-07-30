#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// 172:1 gear ratio = 8256 ticks per full gearbox revolution
// using rising edge only degrees_per_tick = 360.0 / 2064 = 0.1745°

#define MOTOR_POWER 25
#define MOTOR_MAX_VOLTAGE 6 // max voltage is 9, 60% for safety

volatile uint16_t encoder_ticks = 0;

void setup() {
    cli(); // Disable interrupts during setup

    // // Motor direction pin PB6 (digital 12)
    // DDRB |= (1 << DDB6);

    // Motor direction pin PG5 (digital 4)
    DDRG |= (1 << DDG5);

    // PWM pin PE5 (digital 3) output
    DDRE |= (1 << DDE5);

    // Encoder input pin PD0 (INT2) as input
    DDRD &= ~(1 << DDD0);

    // Configure external interrupt INT2 on rising edge
    EICRA |= (1 << ISC21) | (1 << ISC20);  // ISC21:ISC20 = 11 for rising edge
    EIMSK |= (1 << INT2);                   // Enable INT2 interrupt

    // Timer 3 setup for Fast PWM mode 14, TOP = ICR3
    TCCR3A = (1 << COM3C1) | (1 << WGM31);           // Non-inverting mode on OC3C (PE5), WGM31 = 1
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31); // WGM32+WGM33=14 Fast PWM, prescaler=8

    ICR3 = 2000;    // TOP for 1kHz PWM frequency (16MHz / (8*2000))
    OCR3C = (MOTOR_POWER * ICR3) / 100; // Set initial duty cycle based on MOTOR_POWER %

    sei();  // Enable interrupts
}

ISR(INT2_vect) {
    encoder_ticks++;
    if (encoder_ticks >= 2064) {
        // Stop motor PWM by setting duty cycle to 0
        OCR3C = 0;
    }
}

int main(void) {
    setup();

    // Set motor direction pin high or low
    PORTB |= (1 << PORTB6);

    while (1) {
    }
}