#include "SemiDynamicGetup.hpp"

#include <Eigen/Geometry>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/GetUp.hpp"
#include "message/skill/SemiDynamicGetup.hpp"

#include "utility/skill/Script.hpp"


namespace module::skill {

using extension::Configuration;
using SemiDynamicGetupTask = message::skill::SemiDynamicGetup;
using message::actuation::BodySequence;
using utility::skill::load_script;
using message::behaviour::state::Stability;
using message::input::Sensors;

SemiDynamicGetup::SemiDynamicGetup(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

    on<Configuration>("SemiDynamicGetup.yaml").then([this](const Configuration& config) {
        // Use configuration here from file SemiDynamicGetup.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();

        cfg.getup_back        = config["scripts"]["getup_back"].as<std::vector<std::string>>();
    });

    // TODO... fill out template
    // on<Trigger<IMU>>().then([this](const IMU& imu) {
    //     float pitch = imu.orientation.pitch;
    //     update_pitch(pitch);
    // });

    on<Provide<SemiDynamicGetupTask>, Needs<BodySequence>, With<Sensors>>().then([this](const RunReason& run_reason, const Uses<BodySequence>& body, const Sensors& sensors) {
            log<INFO>("SemiDynamicGetup task requested");

            if (!body.done) { // Not gotten up yet
                log<INFO>("Getting up from back within SemiDynamicGetup...");
                emit<Task>(load_script<BodySequence>(cfg.getup_back));
            }
        });

    // on<Trigger<JointState>>().then([this](const JointState& state) {
    //     auto now = NUClear::clock::now();
    //     auto time_in_phase = std::chrono::duration_cast<std::chrono::milliseconds>(now - phase_start).count();

    //     // Example: Phase transition logic based on pitch
    //     if (current_phase == Phase::TORSO_SWING) {
    //         if (pitch_velocity < 0.1f && time_in_phase > 1000) {
    //             transition_to_phase(Phase::KNEE_TUCK);
    //         }
    //     }

    //     // Example: Compute feedback correction
    //     float desired_pitch = 0.5f;  // rad
    //     float pitch_error = desired_pitch - last_pitch;
    //     float correction = Kp * pitch_error - Kd * pitch_velocity;

    //     // Build joint command (simplified)
    //     auto command = std::make_unique<JointCommand>();
    //     command->effort.resize(state.position.size());

    //     for (size_t i = 0; i < state.position.size(); ++i) {
    //         if (utility::input::servo_names[i] == "LEFT_HIP_PITCH"
    //             || utility::input::servo_names[i] == "RIGHT_HIP_PITCH") {
    //             command->effort[i] = correction;  // apply feedback torque
    //         } else {
    //             command->effort[i] = 0.0f;  // placeholder
    //         }
    //     }

    //     emit<Scope::DIRECT>(command);
    // });
}

// void GetUpPitchController::update_pitch(float new_pitch) {
//     static float prev_pitch = last_pitch;
//     auto now = NUClear::clock::now();

//     float dt = 0.01f;  // replace with real dt if needed
//     pitch_velocity = (new_pitch - prev_pitch) / dt;
//     // prev_pitch = last_pitch;
//     last_pitch = new_pitch;
// }

// void GetUpPitchController::transition_to_phase(Phase new_phase) {
//     current_phase = new_phase;
//     phase_start = NUClear::clock::now();
//     log<INFO>("Transitioning to new get-up phase: ", static_cast<int>(new_phase));
// }

}  // namespace module::skill
