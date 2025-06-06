/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "PS3Walk.hpp"

#include <nuclear>
#include <unordered_map>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/skill/Script.hpp"

namespace module::purpose {


    using extension::Configuration;
    using extension::behaviour::Task;

    using message::actuation::LimbsSequence;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::skill::Kick;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;

    using utility::input::LimbID;
    using utility::skill::load_script;

    PS3Walk::PS3Walk(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("PS3Walk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanKick.yaml
            this->log_level                 = config["log_level"].as<NUClear::LogLevel>();
            cfg.maximum_forward_velocity    = config["maximum_forward_velocity"].as<double>();
            cfg.maximum_rotational_velocity = config["maximum_rotational_velocity"].as<double>();
            cfg.max_acceleration            = config["max_acceleration"].as<double>();

            // Load button to script mappings
            for (const auto& script : config["button_scripts"].as<std::map<std::string, std::string>>()) {
                cfg.button_scripts[script.first] = script.second;
            }
        });


        // Start the Director graph for the KeyboardWalk.
        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without these emits, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            // Look forward
            emit<Task>(std::make_unique<Look>(Eigen::Vector3d::UnitX(), true));
        });

        on<Every<1, std::chrono::milliseconds>, Single>().then([this] {
            JoystickEvent event;
            // read from joystick
            if (joystick.sample(&event)) {

                if (event.isAxis()) {
                    // event was an axis event
                    switch (event.number) {
                        case AXIS_LEFT_JOYSTICK_HORIZONTAL:
                            // y is left relative to robot
                            // strafe[1] = -event.value;
                            walk_command.z() = static_cast<double>(-event.value) / std::numeric_limits<short>::max()
                                               * cfg.maximum_rotational_velocity;
                            break;
                        case AXIS_LEFT_JOYSTICK_VERTICAL:
                            // x is forward relative to robot
                            walk_command.x() = static_cast<double>(-event.value) / std::numeric_limits<short>::max()
                                               * cfg.maximum_forward_velocity;
                            break;
                        case AXIS_RIGHT_JOYSTICK_VERTICAL:
                            head_pitch = static_cast<double>(-event.value) / 32767.0;
                            break;
                        case AXIS_RIGHT_JOYSTICK_HORIZONTAL:
                            head_yaw = static_cast<double>(-event.value) / 32767.0;
                            break;
                    }
                }
                else if (event.isButton()) {
                    // event was a button event
                    switch (event.number) {
                        case BUTTON_START:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button START pressed");
                                if (moving) {
                                    log<INFO>("Stop walking");
                                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
                                }
                                else {
                                    log<INFO>("Start walking");
                                }
                                moving = !moving;
                            }
                            break;
                        case BUTTON_SELECT:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button SELECT pressed");
                                if (head_locked) {
                                    log<INFO>("Head unlocked");
                                }
                                else {
                                    log<INFO>("Head locked");
                                    emit<Task>(std::unique_ptr<Look>(nullptr));
                                }
                                head_locked = !head_locked;
                            }
                            break;
                        case BUTTON_DPAD_UP:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button DPAD_UP pressed");
                                if (scripts_enabled && cfg.button_scripts.count("DPAD_UP") > 0) {
                                    const auto& script = cfg.button_scripts.at("DPAD_UP");
                                    log<INFO>("Running script:", script);
                                    emit<Task>(load_script<LimbsSequence>(script), 3);
                                }
                                else if (!scripts_enabled) {
                                    log<INFO>("Scripts are disabled. Enable with R2.");
                                }
                            }
                            break;
                        case BUTTON_DPAD_DOWN:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button DPAD_DOWN pressed");
                                if (scripts_enabled && cfg.button_scripts.count("DPAD_DOWN") > 0) {
                                    const auto& script = cfg.button_scripts.at("DPAD_DOWN");
                                    log<INFO>("Running script:", script);
                                    emit<Task>(load_script<LimbsSequence>(script), 3);
                                }
                                else if (!scripts_enabled) {
                                    log<INFO>("Scripts are disabled. Enable with R2.");
                                }
                            }
                            break;
                        case BUTTON_DPAD_LEFT:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button DPAD_LEFT pressed");
                                if (scripts_enabled && cfg.button_scripts.count("DPAD_LEFT") > 0) {
                                    const auto& script = cfg.button_scripts.at("DPAD_LEFT");
                                    log<INFO>("Running script:", script);
                                    emit<Task>(load_script<LimbsSequence>(script), 3);
                                }
                                else if (!scripts_enabled) {
                                    log<INFO>("Scripts are disabled. Enable with R2.");
                                }
                            }
                            break;
                        case BUTTON_DPAD_RIGHT:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button DPAD_RIGHT pressed");
                                if (scripts_enabled && cfg.button_scripts.count("DPAD_RIGHT") > 0) {
                                    const auto& script = cfg.button_scripts.at("DPAD_RIGHT");
                                    log<INFO>("Running script:", script);
                                    emit<Task>(load_script<LimbsSequence>(script), 3);
                                }
                                else if (!scripts_enabled) {
                                    log<INFO>("Scripts are disabled. Enable with R2.");
                                }
                            }
                            break;
                        case BUTTON_TRIANGLE:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button TRIANGLE pressed");
                                if (scripts_enabled && cfg.button_scripts.count("TRIANGLE") > 0) {
                                    const auto& script = cfg.button_scripts.at("TRIANGLE");
                                    log<INFO>("Running script:", script);
                                    emit<Task>(load_script<LimbsSequence>(script), 3);
                                }
                                else if (!scripts_enabled) {
                                    log<INFO>("Scripts are disabled. Enable with R2.");
                                }
                            }
                            break;
                        case BUTTON_CIRCLE:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button CIRCLE pressed");
                                if (scripts_enabled && cfg.button_scripts.count("CIRCLE") > 0) {
                                    const auto& script = cfg.button_scripts.at("CIRCLE");
                                    log<INFO>("Running script:", script);
                                    emit<Task>(load_script<LimbsSequence>(script), 3);
                                }
                                else if (!scripts_enabled) {
                                    log<INFO>("Scripts are disabled. Enable with R2.");
                                }
                            }
                            break;
                        case BUTTON_CROSS:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button CROSS pressed");
                                if (scripts_enabled && cfg.button_scripts.count("CROSS") > 0) {
                                    const auto& script = cfg.button_scripts.at("CROSS");
                                    log<INFO>("Running script:", script);
                                    emit<Task>(load_script<LimbsSequence>(script), 3);
                                }
                                else if (!scripts_enabled) {
                                    log<INFO>("Scripts are disabled. Enable with R2.");
                                }
                            }
                            break;
                        case BUTTON_SQUARE:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button SQUARE pressed");
                                if (scripts_enabled && cfg.button_scripts.count("SQUARE") > 0) {
                                    const auto& script = cfg.button_scripts.at("SQUARE");
                                    log<INFO>("Running script:", script);
                                    emit<Task>(load_script<LimbsSequence>(script), 3);
                                }
                                else if (!scripts_enabled) {
                                    log<INFO>("Scripts are disabled. Enable with R2.");
                                }
                            }
                            break;
                        case BUTTON_L1:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button L1 pressed");
                                if (scripts_enabled && cfg.button_scripts.count("L1") > 0) {
                                    const auto& action = cfg.button_scripts.at("L1");
                                    if (action == "LEFT_LEG_KICK") {
                                        log<INFO>("Requesting Left Front Kick");
                                        emit<Task>(std::make_unique<Kick>(LimbID::LEFT_LEG), 3);
                                    }
                                    else {
                                        log<INFO>("Running script:", action);
                                        emit<Task>(load_script<LimbsSequence>(action), 3);
                                    }
                                }
                                else if (!scripts_enabled) {
                                    log<INFO>("Scripts are disabled. Enable with R2.");
                                }
                            }
                            break;
                        case BUTTON_R1:
                            if (event.value > 0) {  // button down
                                log<INFO>("Button R1 pressed");
                                if (scripts_enabled && cfg.button_scripts.count("R1") > 0) {
                                    const auto& action = cfg.button_scripts.at("R1");
                                    if (action == "RIGHT_LEG_KICK") {
                                        log<INFO>("Requesting Right Front Kick");
                                        emit<Task>(std::make_unique<Kick>(LimbID::RIGHT_LEG), 3);
                                    }
                                    else {
                                        log<INFO>("Running script:", action);
                                        emit<Task>(load_script<LimbsSequence>(action), 3);
                                    }
                                }
                                else if (!scripts_enabled) {
                                    log<INFO>("Scripts are disabled. Enable with R2.");
                                }
                            }
                            break;
                        case BUTTON_L2:
                            if (event.value > 0) {
                                log<INFO>("Button L2 pressed");
                            }
                            break;
                        case BUTTON_RIGHT_JOYSTICK:
                            if (event.value > 0) {
                                log<INFO>("Button BUTTON_RIGHT_JOYSTICK pressed");
                                if (scripts_enabled) {
                                    log<INFO>("Scripts disabled");
                                }
                                else {
                                    log<INFO>("Scripts enabled");
                                }
                                scripts_enabled = !scripts_enabled;
                            }
                            break;
                    }
                }
            }
            // If it's closed then we should try to reconnect
            else if (!joystick.valid()) {
                joystick.reconnect();
            }
        });

        // output walk command based on updated strafe and rotation speed from joystick
        on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>().then([this] {
            if (!head_locked) {
                // Create a unit vector in the direction the head should be pointing
                Eigen::Vector3d uPCt = (Eigen::AngleAxisd(head_yaw, Eigen::Vector3d::UnitZ())
                                        * Eigen::AngleAxisd(-head_pitch, Eigen::Vector3d::UnitY()))
                                           .toRotationMatrix()
                                       * Eigen::Vector3d::UnitX();
                emit<Task>(std::make_unique<Look>(uPCt, true));
            }

            if (moving) {
                // Apply acceleration limiting
                const double dt = 1.0 / UPDATE_FREQUENCY;

                // Calculate maximum allowed change in velocity components
                double max_delta = cfg.max_acceleration * dt;

                // Create a limited walk command
                Eigen::Vector3d limited_walk_command = walk_command;

                // Limit each component's acceleration
                for (int i = 0; i < 3; i++) {
                    double delta = walk_command[i] - previous_walk_command[i];
                    if (std::abs(delta) > max_delta) {
                        limited_walk_command[i] = previous_walk_command[i] + (delta > 0 ? max_delta : -max_delta);
                    }
                }

                // Store the new command for next cycle
                previous_walk_command = limited_walk_command;

                // Emit the limited walk command
                log<DEBUG>("Walk command:",
                           limited_walk_command.transpose(),
                           " (limited from:",
                           walk_command.transpose(),
                           ")");
                emit<Task>(std::make_unique<Walk>(limited_walk_command), 2);
            }
            else {
                // When not moving, also update previous_walk_command to prevent
                // large accelerations when starting to move again
                previous_walk_command = Eigen::Vector3d::Zero();
            }
        });
    }
}  // namespace module::purpose
