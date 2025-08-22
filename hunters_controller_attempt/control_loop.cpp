#include "controller_hunter.cpp"
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

float final_position = 100.0; // Example final position
float feedrate = 50.0; // Example final feedrate

int main(){
  
  ControllerHunter* controller = new ControllerHunter(final_position, feedrate);

}



ISR(TIMER6_COMPA_vect) {
  // read encoder values
  error = EncoderCountsToDist(encoder_counts) - final_position;
  float output = ControllerHunter->GetControloutput(error);
  clamp (output, -255, 255); // clamp output to motor speed range
  // update motor speed based on control output
  analogWrite(MOTOR_PIN, output);
}

float EncoderCountsToDist(uint16_t encoder_counts ){
  // this function converts encoder counts to a distance in mm
  float shaft_diameter = 13; // shaft diameter in mm
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
  float shaft_diameter = 13; // shaft diameter in mm
  int cpr_before_gear_box = 12; // counts per revolution
  int gear_ratio = 172;
  int cpr_after_gear_box =  cpr_before_gear_box*gear_ratio;
  float counts_to_rad_ratio = 2*M_PI/cpr_after_gear_box;
  float mm_per_count = 0.5*counts_to_rad_ratio*shaft_diameter;
  uint16_t encoder_counts = distance/mm_per_count;
  return encoder_counts;  
}
