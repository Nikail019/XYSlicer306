#include "controller_hunter.hpp"

ControllerHunter::ControllerHunter(float final_position, float feedrate) {
	profile_ = new KinematicProfiler(final_position,feedrate);
	i_ = 0;
}

ControllerHunter::~ControllerHunter() {
	delete profile_;
}

void ControllerHunter::UpdateError(float new_error) {
	previous_error_ = error_;
	error_ = new_error;
	integral_ += new_error;
}

float ControllerHunter::GetControlOutput(float error) {
	UpdateError(error);	
	float feedforward_terms = 0, feedback_terms = 0;
	float Kp = 0, Ki = 0, Kd = 0;
	float Ka = 0, Kv = 0, Ks = 0;
	feedback_terms = Kp * error_ + Ki * integral_ + Kd * (error_ - previous_error_);
	if (i_ < profile_->GetTotalIterations()) {
		feedforward_terms = Ka * profile_->GetAccelerationProfile()[i_] + Kv * profile_->GetVelocityProfile()[i_] + Ks * profile_->GetPositionProfile()[i_];
	}
	float control_output = feedback_terms + feedforward_terms;
	i_++;
	return control_output;

}  