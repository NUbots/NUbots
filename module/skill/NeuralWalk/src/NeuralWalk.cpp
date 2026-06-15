/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 */
#include "NeuralWalk.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/Walk.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"

#include "utility/support/yaml_expression.hpp"
#include <algorithm>
#include <cmath>

namespace module::skill {

    using extension::Configuration;

    using message::actuation::LeftArm;
    using message::actuation::LeftLeg;
    using message::actuation::RightArm;
    using message::actuation::RightLeg;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using WalkTask = message::skill::Walk;

    using utility::input::LimbID;
    using utility::input::ServoID;

    // Same message with a new name!
    struct CurrentWalkTask : WalkTask {
        using WalkTask::WalkTask;
    };

    NeuralWalk::NeuralWalk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("NeuralWalk.yaml").then([this](const Configuration& config) {
            try {
                // Use configuration here from file NeuralWalk.yaml
                this->log_level = config["log_level"].as<NUClear::LogLevel>();

                cfg.model_path = config["model_path"].as<std::string>();
                cfg.device     = config["device"].as<std::string>();
                cfg.step_period = config["step_period"].as<double>();

                // Configure the legs
                for (auto id : utility::input::LimbID::servos_for_legs()) {
                    cfg.servo_states[id] = ServoState(config["gains"]["legs"].as<double>(), 100);
                }
                // Configure the arms
                for (auto id : utility::input::LimbID::servos_for_arms()) {
                    cfg.servo_states[id] = ServoState(config["gains"]["arms"].as<double>(), 100);
                }

                cfg.arm_positions.clear();
                cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_PITCH, config["arms"]["right_shoulder_pitch"].as<double>());
                cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_PITCH, config["arms"]["left_shoulder_pitch"].as<double>());
                cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_ROLL, config["arms"]["right_shoulder_roll"].as<double>());
                cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_ROLL, config["arms"]["left_shoulder_roll"].as<double>());
                cfg.arm_positions.emplace_back(ServoID::R_ELBOW, config["arms"]["right_elbow"].as<double>());
                cfg.arm_positions.emplace_back(ServoID::L_ELBOW, config["arms"]["left_elbow"].as<double>());

                // Since walk needs a Stability message to run, emit one at the beginning
                emit(std::make_unique<Stability>(Stability::UNKNOWN));

                // Compile the model and create inference request object
                log<INFO>("Loading NeuralWalk ONNX model from: ", cfg.model_path);
                log<INFO>("Using device: ", cfg.device);

                ov::Core core{};

                // Try to fallback to CPU if GPU fails
                try {
                    compiled_model = core.compile_model(cfg.model_path, cfg.device);
                }
                catch (const std::exception& e) {
                    if (cfg.device == "GPU") {
                        log<WARN>("Failed to compile model on GPU, falling back to CPU: ", e.what());
                        compiled_model = core.compile_model(cfg.model_path, "CPU");
                    }
                    else {
                        throw;
                    }
                }

                infer_request = compiled_model.create_infer_request();
                log<INFO>("NeuralWalk ONNX model loaded successfully");

                // Initialize action history (3 frames, 12 leg joints each = 36)
                action_history.assign(36, 0.0f);
            } catch (const std::exception& e) {
                log<ERROR>("Failed to configure NeuralWalk or load model: ", e.what());
            }
        });

        // Start - Runs every time the Walk provider starts (wasn't running)
        on<Start<WalkTask>>().then([this]() {
            log<INFO>("NeuralWalk task started!");
            // Reset the last update time and walk engine state
            last_update_time = NUClear::clock::now();
            phase_time = 0.0;
            phase_indicator = 1.0;
            action_history.assign(36, 0.0f);

            // Emit a zero current walk command
            emit(std::make_unique<CurrentWalkTask>());

            // Emit a stopped state as we are not yet walking
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
        });

        // Stop - Runs every time the Walk task is removed from the director tree
        on<Stop<WalkTask>>().then([this] {
            log<INFO>("NeuralWalk task stopped!");
            // Emit a stopped state as we are now not walking
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
        });

        // Main loop - Updates the neural walk engine at fixed frequency
        on<Provide<WalkTask>,
           Needs<LeftLeg>,
           Needs<RightLeg>,
           Needs<LeftArm>,
           Needs<RightArm>,
           With<CurrentWalkTask>,
           With<Sensors>,
           With<Stability>,
           Single,
           Every<100, Per<std::chrono::seconds>>,
           Priority::HIGH>()
            .then([this](const WalkTask& new_walk,
                         const RunReason& run_reason,
                         const CurrentWalkTask& current_walk,
                         const Sensors& sensors,
                         const Stability& stability) {
               // Force the walk to run at a known rate
                if (run_reason != RunReason::OTHER_TRIGGER)
                    return;

                // Always update the current walk task with the latest request
                auto walk = new_walk;
                emit(std::make_unique<CurrentWalkTask>(walk));

                // Compute time since the last update
                auto time_delta =
                    std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - last_update_time)
                        .count();
                last_update_time = NUClear::clock::now();

                if (stability == Stability::FALLEN) {
                    log<DEBUG>("Stability is FALLEN, returning");
                    return;
                }

                // Smooth velocity target (prevent instant jumps)
                Eigen::Vector3d dv = cfg.acceleration * std::min(time_delta, 1.0);
                current_velocity = current_velocity + (walk.velocity_target - current_velocity).cwiseMax(-dv).cwiseMin(dv);

                // Emulate WalkEngine state machine EXACTLY as it is in WalkGenerator.hpp
                bool is_velocity_zero = (current_velocity.norm() < 0.01);
                
                static WalkState::State engine_state = WalkState::State::STOPPED;

                if (is_velocity_zero && phase_time < cfg.step_period) {
                    engine_state = WalkState::State::STOPPING;
                } else if (phase_time >= cfg.step_period && (is_velocity_zero || engine_state == WalkState::State::STOPPING)) {
                    engine_state = WalkState::State::STOPPED;
                } else if (!is_velocity_zero && engine_state == WalkState::State::STOPPED) {
                    engine_state = WalkState::State::STARTING;
                    phase_time = 0.0;
                }

                // Advance internal phase clock only if not fully stopped
                if (engine_state != WalkState::State::STOPPED) {
                    phase_time += time_delta;
                } else {
                    phase_time = cfg.step_period; // WalkGenerator uses t = step_period when stopped!
                }

                // Handle state transitions that occur at the end of the step
                if (engine_state == WalkState::State::STARTING) {
                    if (phase_time >= cfg.step_period) {
                        engine_state = WalkState::State::WALKING;
                        phase_time -= cfg.step_period;
                        // WalkGenerator does NOT switch the planted foot here!
                    }
                } else if (engine_state == WalkState::State::WALKING) {
                    if (phase_time >= cfg.step_period) {
                        phase_time -= cfg.step_period;
                        phase_indicator *= -1.0; // Switch planted foot only in WALKING state
                    }
                }

                float eng_stop = (engine_state == WalkState::State::STOPPED) ? 1.0f : 0.0f;
                float eng_start = (engine_state == WalkState::State::STARTING) ? 1.0f : 0.0f;
                float eng_walk = (engine_state == WalkState::State::WALKING) ? 1.0f : 0.0f;
                float eng_stop_trans = (engine_state == WalkState::State::STOPPING) ? 1.0f : 0.0f;

                double phase_ratio = cfg.step_period > 0 ? (phase_time / cfg.step_period) : 0.0;
                double phase_sin = std::sin(2.0 * M_PI * phase_ratio);
                double phase_cos = std::cos(2.0 * M_PI * phase_ratio);

                // 1. Gather Walk commands (vx, vy, vtheta)
                float vx = current_velocity.x();
                float vy = current_velocity.y();
                float vt = current_velocity.z();

                // 2. Populate OpenVINO input tensor (46 dims)
                ov::Tensor input_tensor;
                try {
                    input_tensor = infer_request.get_input_tensor();
                    input_tensor.set_shape(ov::Shape{1, 46});
                } catch (const std::exception& e) {
                    log<ERROR>("Failed to get input tensor: ", e.what());
                    return;
                }

                float* input_data = nullptr;
                try {
                    input_data = input_tensor.data<float>();
                } catch (const std::exception& e) {
                    log<ERROR>("Failed to get data pointer: ", e.what());
                    return;
                }

                // Fill tensor
                input_data[0] = vx;
                input_data[1] = vy;
                input_data[2] = vt;
                input_data[3] = phase_sin;
                input_data[4] = phase_cos;
                input_data[5] = phase_indicator;
                input_data[6] = eng_stop;
                input_data[7] = eng_start;
                input_data[8] = eng_walk;
                input_data[9] = eng_stop_trans;


                // Copy history
                std::copy(action_history.begin(), action_history.end(), input_data + 10);

                // 4. Run inference
                infer_request.set_input_tensor(input_tensor);
                try {
                    infer_request.infer();
                } catch (const std::exception& e) {
                    log<DEBUG>("Inference failed: ", e.what());
                    return;
                }

                auto output_tensor = infer_request.get_output_tensor(0);
                float* output_data = output_tensor.data<float>();

                // Ensure action_history is valid size
                if (action_history.size() < 36) {
                    action_history.assign(36, 0.0f);
                    
                    std::vector<ServoID> left_ids = {
                        ServoID::L_HIP_YAW, ServoID::L_HIP_ROLL, ServoID::L_HIP_PITCH,
                        ServoID::L_KNEE, ServoID::L_ANKLE_PITCH, ServoID::L_ANKLE_ROLL
                    };
                    std::vector<ServoID> right_ids = {
                        ServoID::R_HIP_YAW, ServoID::R_HIP_ROLL, ServoID::R_HIP_PITCH,
                        ServoID::R_KNEE, ServoID::R_ANKLE_PITCH, ServoID::R_ANKLE_ROLL
                    };
                    
                    for (int h = 0; h < 3; ++h) {
                        for (int i = 0; i < 6; ++i) {
                            action_history[h * 12 + i] = sensors.servo[left_ids[i]].present_position;
                            action_history[h * 12 + 6 + i] = sensors.servo[right_ids[i]].present_position;
                        }
                    }
                }

                // 5. Shift action_history and append new predictions backwards
                for (int i = 23; i >= 0; --i) {
                    action_history[12 + i] = action_history[i];
                }
                for (int i = 0; i < 12; ++i) {
                    action_history[i] = output_data[i];
                }

                // 6. Emit ServoTargets
                const NUClear::clock::time_point goal_time = NUClear::clock::now() + std::chrono::milliseconds(10);

                auto left_leg = std::make_unique<LeftLeg>();
                auto right_leg = std::make_unique<RightLeg>();

                // Order of joints in distillation: Left Leg (6), Right Leg (6)
                // Left leg: Hip Yaw, Hip Roll, Hip Pitch, Knee, Ankle Pitch, Ankle Roll
                std::vector<ServoID> left_leg_ids = {
                    ServoID::L_HIP_YAW, ServoID::L_HIP_ROLL, ServoID::L_HIP_PITCH,
                    ServoID::L_KNEE, ServoID::L_ANKLE_PITCH, ServoID::L_ANKLE_ROLL
                };

                for (size_t i = 0; i < left_leg_ids.size(); ++i) {
                    left_leg->servos[left_leg_ids[i]] = ServoCommand(goal_time, output_data[i], cfg.servo_states[left_leg_ids[i]]);
                }

                std::vector<ServoID> right_leg_ids = {
                    ServoID::R_HIP_YAW, ServoID::R_HIP_ROLL, ServoID::R_HIP_PITCH,
                    ServoID::R_KNEE, ServoID::R_ANKLE_PITCH, ServoID::R_ANKLE_ROLL
                };

                for (size_t i = 0; i < right_leg_ids.size(); ++i) {
                    right_leg->servos[right_leg_ids[i]] = ServoCommand(goal_time, output_data[6 + i], cfg.servo_states[right_leg_ids[i]]);
                }

                emit<Task>(left_leg);
                emit<Task>(right_leg);

                // Arms (keep them at a fixed position like Walk.cpp does)
                auto left_arm = std::make_unique<LeftArm>();
                auto right_arm = std::make_unique<RightArm>();

                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::RIGHT_ARM)) {
                    // Find position in config
                    auto it = std::find_if(cfg.arm_positions.begin(), cfg.arm_positions.end(), [id](const auto& pair) {
                        return pair.first == id;
                    });
                    if (it != cfg.arm_positions.end()) {
                        right_arm->servos[id] = ServoCommand(goal_time, it->second, cfg.servo_states[id]);
                    }
                }

                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::LEFT_ARM)) {
                    auto it = std::find_if(cfg.arm_positions.begin(), cfg.arm_positions.end(), [id](const auto& pair) {
                        return pair.first == id;
                    });
                    if (it != cfg.arm_positions.end()) {
                        left_arm->servos[id] = ServoCommand(goal_time, it->second, cfg.servo_states[id]);
                    }
                }

                emit<Task>(left_arm, 0, true, "NeuralWalk left arm");
                emit<Task>(right_arm, 0, true, "NeuralWalk right arm");

                // Emit walk state
                emit(std::make_unique<WalkState>(
                    walk.velocity_target.norm() < 0.01 ? WalkState::State::STOPPED : WalkState::State::WALKING,
                    walk.velocity_target
                ));
            });
    }

}  // namespace module::skill
