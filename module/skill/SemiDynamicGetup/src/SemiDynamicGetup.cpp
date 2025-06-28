#include "SemiDynamicGetup.hpp"

#include "extension/Configuration.hpp"

namespace module::skill {

using extension::Configuration;
using Phase = SemiDynamicGetup::Phase;

SemiDynamicGetup::SemiDynamicGetup(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("SemiDynamicGetup.yaml").then([this](const Configuration& config) {
        // Use configuration here from file SemiDynamicGetup.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });

    // TODO... fill out template
    on<Trigger<IMU>>().then([this](const IMU& imu) {
        float pitch = imu.orientation.pitch;
        update_pitch(pitch);
    });

    on<Trigger<JointState>>().then([this](const JointState& state) {
        auto now = NUClear::clock::now();
        auto time_in_phase = std::chrono::duration_cast<std::chrono::milliseconds>(now - phase_start).count();

        // Example: Phase transition logic based on pitch
        if (current_phase == Phase::TORSO_SWING) {
            if (pitch_velocity < 0.1f && time_in_phase > 1000) {
                transition_to_phase(Phase::KNEE_TUCK);
            }
        }

        // Example: Compute feedback correction
        float desired_pitch = 0.5f;  // rad
        float pitch_error = desired_pitch - last_pitch;
        float correction = Kp * pitch_error - Kd * pitch_velocity;

        // Build joint command (simplified)
        auto command = std::make_unique<JointCommand>();
        command->effort.resize(state.position.size());

        for (size_t i = 0; i < state.position.size(); ++i) {
            if (utility::input::servo_names[i] == "LEFT_HIP_PITCH"
                || utility::input::servo_names[i] == "RIGHT_HIP_PITCH") {
                command->effort[i] = correction;  // apply feedback torque
            } else {
                command->effort[i] = 0.0f;  // placeholder
            }
        }

        emit<Scope::DIRECT>(command);
    });

    on<Startup>().then([this] {
        current_phase = Phase::LAYING;
        phase_start = NUClear::clock::now();
        log<INFO>("GetUpPitchController started.");
    });
}

void GetUpPitchController::update_pitch(float new_pitch) {
    static float prev_pitch = last_pitch;
    auto now = NUClear::clock::now();

    float dt = 0.01f;  // replace with real dt if needed
    pitch_velocity = (new_pitch - prev_pitch) / dt;
    // prev_pitch = last_pitch;
    last_pitch = new_pitch;
}

void GetUpPitchController::transition_to_phase(Phase new_phase) {
    current_phase = new_phase;
    phase_start = NUClear::clock::now();
    log<INFO>("Transitioning to new get-up phase: ", static_cast<int>(new_phase));
}

}  // namespace module::skill
