#include "kinematic_profiler.hpp"

KinematicProfiler::KinematicProfiler(float final_position, float feedrate) :
    final_position_(final_position), feedrate_(feedrate) {
    GenerateProfiles(final_position, feedrate);
}

void KinematicProfiler::GenerateProfiles(float final_position, float feedrate) {
    accel_time_ = feedrate_ / A_max_;
    float accel_distance = 0.5 * (feedrate_ * feedrate_) / A_max_;
    float cruise_distance = final_position - 2 * accel_distance;
    cruise_time_ = cruise_distance / feedrate;
    float total_time = cruise_time_ + 2 * accel_time_;
    total_iterations_ = total_time / sampling_period_;
    if (cruise_distance <= 0) {
        // switch to triangle profile logic
        cruise_distance = 0;
        cruise_time_ = 0;
        GenerateAccelerationTriProfile();
        GenerateVelocityTriProfile();
        GeneratePositionTriProfile();
    }
    else {
        // continue with trapezium profile logic
        cruise_time_ = cruise_distance / feedrate;
        GenerateAccelerationTrapProfile();
        GenerateVelocityTrapProfile();
        GeneratePositionTrapProfile();
    }
}

void KinematicProfiler::GenerateAccelerationTrapProfile() {
    for (int i = 0; i < total_iterations_; i++) {
        float t = i * sampling_period_;
        if (t < accel_time_) {
            acceleration_profile_.push_back(A_max_);
        }
        else if (t > (accel_time_ + cruise_time_)) {
            acceleration_profile_.push_back(-A_max_);
        }
        else {
            acceleration_profile_.push_back(0);
        }
    }
}

void KinematicProfiler::GenerateVelocityTrapProfile() {
    for (int i = 0; i < total_iterations_; i++) {
        float t = i * sampling_period_;
        if (t < accel_time_) {
            velocity_profile_.push_back(A_max_ * t);
        }
        else if (t > (accel_time_ + cruise_time_)) {
            velocity_profile_.push_back(feedrate_ - A_max_ * (t - (cruise_time_ + accel_time_)));
        }
        else {
            velocity_profile_.push_back(feedrate_);
        }
    }
}

void KinematicProfiler::GeneratePositionTrapProfile() {
    float x_accel_end = 0.5 * A_max_ * accel_time_ * accel_time_;
    float x_cruise_end = x_accel_end + feedrate_ * cruise_time_;
    
    for (int i = 0; i < total_iterations_; i++) {
        float t = i * sampling_period_;
        if (t < accel_time_) {
            position_profile_.push_back(0.5 * A_max_ * t * t);
        }
        else if (t > (accel_time_ + cruise_time_)) {
            float t_dec = t - (accel_time_ + cruise_time_);
            position_profile_.push_back(x_cruise_end + feedrate_ * t_dec - 0.5 * A_max_ * t_dec * t_dec);
        }
        else {
            float x_accel_end = 0.5 * A_max_ * accel_time_ * accel_time_;
            position_profile_.push_back(x_accel_end + feedrate_ * (t - accel_time_));
        }
    }
}

void KinematicProfiler::GenerateAccelerationTriProfile() {
    for (int i = 0; i < total_iterations_; i++) {
        float t = i * sampling_period_;
        if (i < total_iterations_ / 2) {
            acceleration_profile_.push_back(A_max_);
        }
        else {
            acceleration_profile_.push_back(-A_max_);
        }
    }
}

void KinematicProfiler::GenerateVelocityTriProfile() {
    for (int i = 0; i < total_iterations_; i++) {
        float t = i * sampling_period_;
        float mid_time = sampling_period_ * total_iterations_ / 2;
        if (i < total_iterations_ / 2) {
            velocity_profile_.push_back(A_max_ * t);
        }
        else {
            velocity_profile_.push_back(A_max_ * mid_time - A_max_ * (t - (mid_time)));
        }
    }
}

void KinematicProfiler::GeneratePositionTriProfile() {
    float x_accel_end = 0.5 * A_max_ * accel_time_ * accel_time_;
  
    float mid_time = sampling_period_ * total_iterations_ / 2;
    for (int i = 0; i < total_iterations_; i++) {
        float t = i * sampling_period_;
        if (i < total_iterations_ / 2) {
            position_profile_.push_back(0.5 * A_max_ * t * t);
        }
        else {
            float t_dec = t - mid_time;
            position_profile_.push_back(0.5 * A_max_ * mid_time * mid_time + feedrate_ * t_dec - 0.5 * A_max_ * t_dec * t_dec);
        }
    }
}
/*
void KinematicProfiler::ExportAccelerationProfile(const std::string& filename) const {
    std::ofstream file(filename); // Open the file for writing
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing.\n";
        return;
    }

    file << "total iterations ," << total_iterations_ << "\n\n\n";

    // Option 1: One value per line
    for (float a : acceleration_profile_) {
        file << a << "\n";
    }

    // Option 2: All values in one line, comma-separated
    
    // for (size_t i = 0; i < acceleration_profile_.size(); i++) {
    //     file << acceleration_profile_[i];
    //     if (i != acceleration_profile_.size() - 1) file << ",";
    // }
    // file << "\n";
    

    file.close();
    std::cout << "Acceleration profile exported to " << filename << "\n";
}
*/
// void KinematicProfiler::ExportVelocityProfile(const std::string& filename) const {
//     std::ofstream file(filename, std::ios::app); // Open the file for writing
//     if (!file.is_open()) {
//         std::cerr << "Error: Could not open file " << filename << " for writing.\n";
//         return;
//     }

//     file << "total iterations ," << total_iterations_ << "\n\n\n";

//     // Option 1: One value per line
//     //for (float a : velocity_profile_) {
//     //    file << a << "\n";
//     //}

//     // Option 2: All values in one line, comma-separated
    
//     for (size_t i = 0; i < acceleration_profile_.size(); i++) {
//         file << acceleration_profile_[i];
//         if (i != acceleration_profile_.size() - 1) file << ",";
//     }
//     file << "\n";
//     for (size_t i = 0; i < velocity_profile_.size(); i++) {
//         file << velocity_profile_[i];
//         if (i != velocity_profile_.size() - 1) file << ",";
//     }
//     file << "\n";
//     for (size_t i = 0; i < position_profile_.size(); i++) {
//         file << position_profile_[i];
//         if (i != position_profile_.size() - 1) file << ",";
//     }
//     file << "\n";
    

//     file.close();
//     std::cout << "Velocity profile exported to " << filename << "\n";
// }

// void KinematicProfiler::ExportPositionProfile(const std::string& filename) const {
//     std::ofstream file(filename); // Open the file for writing
//     if (!file.is_open()) {
//         std::cerr << "Error: Could not open file " << filename << " for writing.\n";
//         return;
//     }

//     file << "total iterations ," << total_iterations_ << "\n\n\n";

//     // Option 1: One value per line
//     for (float a : position_profile_) {
//         file << a << "\n";
//     }

//     // Option 2: All values in one line, comma-separated
//     /*
//     for (size_t i = 0; i < acceleration_profile_.size(); i++) {
//         file << acceleration_profile_[i];
//         if (i != acceleration_profile_.size() - 1) file << ",";
//     }
//     file << "\n";
//     */

//     file.close();
//     std::cout << "Position profile exported to " << filename << "\n";
// }

const float KinematicProfiler::GetFinalPosition() const {
    return final_position_;
}

const float KinematicProfiler::GetFeedRate() const {
    return feedrate_;
}

const int KinematicProfiler::GetTotalIterations() const {
    return total_iterations_;
}

const std::vector<float>& KinematicProfiler::GetAccelerationProfile() const {

    return acceleration_profile_;
}

const std::vector<float>& KinematicProfiler::GetVelocityProfile() const {
    return velocity_profile_;
}

const std::vector<float>& KinematicProfiler::GetPositionProfile() const {
    return position_profile_;
}