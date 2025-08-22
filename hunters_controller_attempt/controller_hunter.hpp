#include "kinematic_profiler.hpp"

class ControllerHunter {
public:
	explicit ControllerHunter(float final_position, float feedrate);
	~ControllerHunter();
	float GetControlOutput(float error);
	void UpdateError(float new_error);
private:
	KinematicProfiler* profile_;
	int i_;
	float error_;
	float integral_;
	float previous_error_;

};