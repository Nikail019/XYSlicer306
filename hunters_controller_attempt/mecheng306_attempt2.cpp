// mecheng306_attempt2.cpp : Defines the entry point for the application.
//

#include "mecheng306_attempt2.h"
#include <iostream>
#include <vector>
#include "kinematic_profiler.hpp"

int main() {
	for (int i = 0; i < 20; i++) {
		KinematicProfiler profiler(i * 10, 200 - i * 10); // final position, feedrate

		profiler.ExportAccelerationProfile("acceleration_profile.csv");
		profiler.ExportVelocityProfile("velocity_profile.csv");
		profiler.ExportPositionProfile("Position_profile.csv");
		/*profiler.ExportAccelerationProfile("C:/Users/hunte/Desktop/acceleration_profile.csv");
		profiler.ExportVelocityProfile("C:/Users/hunte/Desktop/velocity_profile.csv");*/
	}
}

