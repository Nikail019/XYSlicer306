#include <vector>
#include <iostream>
// #include <fstream>
#include <cmath>

class KinematicProfiler {
public:
	explicit KinematicProfiler(float final_position, float feedrate);
	const std::vector<float>& GetAccelerationProfile() const;
	const std::vector<float>& GetVelocityProfile() const;
	const std::vector<float>& GetPositionProfile() const;
	const float GetFinalPosition() const;
	const float GetFeedRate() const;
	const int GetTotalIterations() const;
	void GenerateProfiles(float final_position, float feedrate);
	void GenerateAccelerationTrapProfile();
	void GenerateVelocityTrapProfile();
	void GeneratePositionTrapProfile();
	void GenerateAccelerationTriProfile();
	void GenerateVelocityTriProfile();
	void GeneratePositionTriProfile();

	void ExportAccelerationProfile(const std::string& filename) const;
	void ExportVelocityProfile(const std::string& filename) const;
	void ExportPositionProfile(const std::string& filename) const;

private:
	float final_position_;
	float feedrate_;
	float cruise_time_;
	float accel_time_;
	float sampling_period_ = 0.001;
	float A_max_ = 200; // find this from motor 
	int total_iterations_;
	std::vector<float> acceleration_profile_;
	std::vector<float> velocity_profile_;
	std::vector<float> position_profile_;

};
