#include <avr/io.h>
#include <util/delay.h>
//172:1 gear ratio = 8256 ticks per full gearbox revolution
//using rising edge only degrees_per_tick = 360.0 / 2064 = 0.1745°

#define MOTOR_POWER 25
#define MOTOR_MAX_VOLTAGE 6 //max voltage is 9 60% for safety

//d12 motor direction High/Low
//pwm d3 motor speed

DDRB |= (1 << DDB6);  // Set PB6 (digital pin 12) as output
DDRE |= (1 << DDE5); // set pin 3 to output
// set pin d21 as interupt for counting encoder
DDRD &= ~(1 << DDD2);
//rising edge
EICRA |= (1 << ISC21) | (1 << ISC20);
//Enable INT2
EIMSK |= (1 << INT2);

// timer 3 setup

