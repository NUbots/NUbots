/*
 * This file is part of PS3Walk.
 *
 * PS3Walk is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PS3Walk is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PS3Walk.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#include "PS3Walk.hpp"

#include <nuclear>

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/behaviour/state/Stability.hpp"
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
    using message::skill::Kick;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;
    using utility::skill::load_script;

    using extension::Configuration;
    using message::behaviour::state::Stability;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;
    using NUClear::message::LogMessage;
    using utility::input::LimbID;

    PS3Walk::PS3Walk(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("PS3Walk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanKick.yaml
            this->log_level                 = config["log_level"].as<NUClear::LogLevel>();
            cfg.maximum_forward_velocity    = config["maximum_forward_velocity"].as<double>();
            cfg.maximum_rotational_velocity = config["maximum_rotational_velocity"].as<double>();
        });


        // Start the Director graph for the KeyboardWalk.
        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without this emit, modules that need a Stability message may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
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
                            head_pitch = static_cast<double>(-event.value) / 32767.0f;
                            break;
                        case AXIS_RIGHT_JOYSTICK_HORIZONTAL:
                            head_yaw = static_cast<double>(-event.value) / 32767.0f;
                            break;
                    }
                }
                else if (event.isButton()) {
                    // event was a button event
                    switch (event.number) {
                        case BUTTON_START:
                            if (event.value > 0) {  // button down
                                if (moving) {
                                    log<NUClear::DEBUG>("Stop walking");
                                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 2);
                                    emit<Task>(std::unique_ptr<Walk>(nullptr));
                                }
                                else {
                                    log<NUClear::DEBUG>("Start walking");
                                }
                                moving = !moving;
                            }
                            break;
                        case BUTTON_SELECT:
                            if (event.value > 0) {  // button down
                                if (head_locked) {
                                    NUClear::log("Head unlocked");
                                }
                                else {
                                    NUClear::log("Head locked");
                                }
                                head_locked = !head_locked;
                            }
                            break;
                            // dance moves here:
                        case BUTTON_DPAD_UP:
                            if (event.value > 0) {
                                NUClear::log("Do a dance move Dpad up");
                                emit<Task>(load_script<LimbsSequence>("StepClap1.yaml"), 3);
                                emit<Task>(std::unique_ptr<Walk>(nullptr));
                            }
                            break;
                        case BUTTON_DPAD_DOWN:
                            if (event.value > 0) {
                                NUClear::log("Do a dance Dpad down");
                                emit<Task>(std::unique_ptr<Walk>(nullptr));
                                emit<Task>(load_script<LimbsSequence>("StepClap2.yaml"), 3);
                            }
                            break;
                        case BUTTON_DPAD_LEFT:
                            if (event.value > 0) {
                                NUClear::log("Do a dance Dpad left");
                                emit<Task>(std::unique_ptr<Walk>(nullptr));
                                emit<Task>(load_script<LimbsSequence>("OverheadThrustRight.yaml"), 3);
                            }
                            break;
                        case BUTTON_DPAD_RIGHT:
                            if (event.value > 0) {
                                NUClear::log("Do a dance Dpad right");
                                emit<Task>(std::unique_ptr<Walk>(nullptr));
                                emit<Task>(load_script<LimbsSequence>("OverheadThrustLeft.yaml"), 3);
                            }
                            break;
                        case BUTTON_TRIANGLE:
                            if (event.value > 0) {
                                NUClear::log("Do a dance triangle");
                                emit<Task>(std::unique_ptr<Walk>(nullptr));
                                emit<Task>(load_script<LimbsSequence>("Star1.yaml"), 3);
                            }
                            break;
                        case BUTTON_CIRCLE:
                            if (event.value > 0) {
                                NUClear::log("Do a dance circle");
                                emit<Task>(std::unique_ptr<Walk>(nullptr));
                                emit<Task>(load_script<LimbsSequence>("Star2.yaml"), 3);
                            }
                            break;
                        case BUTTON_CROSS:
                            if (event.value > 0) {
                                NUClear::log("Do a dance cross");
                                emit<Task>(std::unique_ptr<Walk>(nullptr));
                                emit<Task>(load_script<LimbsSequence>("Crouch1.yaml"), 3);
                            }
                            break;
                        case BUTTON_SQUARE:
                            if (event.value > 0) {
                                NUClear::log("Do a dance square");
                                emit<Task>(std::unique_ptr<Walk>(nullptr));
                                emit<Task>(load_script<LimbsSequence>("Crouch2.yaml"), 3);
                            }
                            break;
                        case BUTTON_L1:
                            if (event.value > 0) {
                                NUClear::log("Requesting Left Front Kick");
                                emit<Task>(std::unique_ptr<Walk>(nullptr));
                                emit<Task>(std::make_unique<Kick>(LimbID::LEFT_LEG), 3);
                            }
                            break;
                        case BUTTON_R1:
                            if (event.value > 0) {
                                NUClear::log("Requesting Right Front Kick");
                                emit<Task>(std::unique_ptr<Walk>(nullptr));
                                emit<Task>(std::make_unique<Kick>(LimbID::RIGHT_LEG), 3);
                            }
                            break;
                            // case BUTTON_L2:
                            //     if (event.value > 0) {  // button down
                            //         NUClear::log("Requesting Left Side Kick");
                            //         emit(std::make_unique<KickScriptCommand>(KickScriptCommand(
                            //             -Eigen::Vector3d::UnitY(),  // vector pointing right relative to
                            //             robot LimbID::LEFT_LEG)));
                            //     }
                            //     break;
                            // case BUTTON_R2:
                            //     if (event.value > 0) {  // button down
                            //         NUClear::log("Requesting Right Side Kick");
                            //         emit(std::make_unique<KickScriptCommand>(KickScriptCommand{
                            //             Eigen::Vector3d::UnitY(),  // vector pointing left relative to robot
                            //             LimbID::RIGHT_LEG}));
                            //     }
                            //     break;
                    }
                }
            }
            // If it's closed then we should try to reconnect
            else if (!joystick.valid()) {
                joystick.reconnect();
            }
        });

        // output walk command based on updated strafe and rotation speed from joystick
        on<Every<20, Per<std::chrono::seconds>>>().then([this] {
            if (!head_locked) {
                // Create a unit vector in the direction the head should be pointing
                Eigen::Vector3d uPCt = (Eigen::AngleAxisd(head_yaw, Eigen::Vector3d::UnitZ())
                                        * Eigen::AngleAxisd(-head_pitch, Eigen::Vector3d::UnitY()))
                                           .toRotationMatrix()
                                       * Eigen::Vector3d::UnitX();
                emit<Task>(std::make_unique<Look>(uPCt, true));
            }

            if (moving) {
                emit<Task>(std::make_unique<Walk>(walk_command.cast<double>()), 2);
            }
        });
    }
}  // namespace module::purpose
