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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "PS3Walk.hpp"

#include <nuclear>

#include "message/behaviour/MotionCommand.hpp"
#include "message/behaviour/ServoCommand.hpp"
#include "message/motion/HeadCommand.hpp"
#include "message/motion/KickCommand.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"

namespace module::behaviour::strategy {

    using message::behaviour::MotionCommand;
    using message::motion::HeadCommand;
    using message::motion::KickCommand;
    using message::motion::KickCommandType;
    using message::motion::KickScriptCommand;
    using utility::input::LimbID;


    PS3Walk::PS3Walk(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

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
                            rotationalSpeed = static_cast<float>(-event.value);
                            log<NUClear::INFO>("Left Joystick Horizontal: ", -event.value);
                            break;
                        case AXIS_LEFT_JOYSTICK_VERTICAL:
                            // x is forward relative to robot
                            strafe[0] = -event.value;
                            log<NUClear::INFO>("Left Joystick Vertical: ", -event.value);
                            break;
                        case AXIS_RIGHT_JOYSTICK_VERTICAL:
                            headPitch = static_cast<float>(-event.value);
                            log<NUClear::INFO>("Right Joystick Vertical: ", -event.value);
                            break;

                        case AXIS_RIGHT_JOYSTICK_HORIZONTAL:
                            headYaw = static_cast<float>(-event.value);
                            log<NUClear::INFO>("Right Joystick Horizontal: ", -event.value);
                            break;
                    }
                }
                else if (event.isButton()) {
                    // event was a button event
                    switch (event.number) {
                        case BUTTON_TRIANGLE:
                            if (event.value > 0) {  // button down
                                if (moving) {
                                    NUClear::log("Stop walking");
                                    emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                                }
                                else {
                                    NUClear::log("Start walking");
                                }
                                moving = !moving;
                            }
                            break;

                        case BUTTON_SQUARE:
                            if (event.value > 0) {  // button down
                                if (headLocked) {
                                    log<NUClear::INFO>("Square button: Head Unlocked");
                                }
                                else {
                                    log<NUClear::INFO>("Square button: Head locked");
                                }
                                headLocked = !headLocked;
                            }
                            break;

                        case BUTTON_CROSS:
                            if (event.value > 0) {  // button down
                                log<NUClear::INFO>("Cross button: Unassigned");
                            }
                            break;

                        case BUTTON_CIRCLE:
                            if (event.value > 0) {  // button down
                                log<NUClear::INFO>("Circle button: Unassigned");
                            }
                            break;

                        case BUTTON_L1:
                            if (event.value > 0) {  // button down
                                log<NUClear::INFO>("L1 Button: Left Kick");
                                emit(std::make_unique<KickScriptCommand>(LimbID::LEFT_LEG, KickCommandType::NORMAL));
                            }
                            break;

                        case BUTTON_R1:
                            if (event.value > 0) {  // button down
                                log<NUClear::INFO>("R1 Button: Right Kick");
                                emit(std::make_unique<KickScriptCommand>(LimbID::RIGHT_LEG, KickCommandType::NORMAL));
                            }
                            break;

                        case BUTTON_L2:
                            if (event.value > 0) {  // button down
                                log<NUClear::INFO>("L2 Button: Unassigned");
                            }
                            break;

                        case BUTTON_R2:
                            if (event.value > 0) {  // button down
                                log<NUClear::INFO>("R2 Button: Unassigned");
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
        // TODO(HardwareTeam): potential performance gain: ignore if value hasn't changed since last emit?
        on<Every<20, Per<std::chrono::seconds>>>().then([this] {
            if (!headLocked) {
                auto headCommand         = std::make_unique<HeadCommand>();
                headCommand->yaw         = headYaw / std::numeric_limits<short>::max() * 1.5f;
                headCommand->pitch       = headPitch / std::numeric_limits<short>::max();
                headCommand->robot_space = true;
                emit(std::move(headCommand));
            }

            if (moving) {
                const auto strafeNorm          = strafe / std::numeric_limits<short>::max();
                const auto rotationalSpeedNorm = rotationalSpeed / std::numeric_limits<short>::max();
                Eigen::Affine2d affineParameter;
                affineParameter.linear()      = Eigen::Rotation2Dd(rotationalSpeedNorm).toRotationMatrix();
                affineParameter.translation() = Eigen::Vector2d(strafeNorm.x(), strafeNorm.y());
                emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(affineParameter)));
            }
        });
    }
}  // namespace module::behaviour::strategy
