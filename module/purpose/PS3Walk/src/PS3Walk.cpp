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
#include "message/actuation/Limbs.hpp"

#include "utility/skill/Script.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "utility/behaviour/Action.hpp"
#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"

namespace module::behaviour::strategy {

    using message::behaviour::MotionCommand;
    using extension::behaviour::Task;
    using message::motion::HeadCommand;
    using message::actuation::BodySequence;
    using message::skill::Look;
    using message::skill::Walk;
    using utility::skill::load_script;


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
                            break;
                        case AXIS_LEFT_JOYSTICK_VERTICAL:
                            // x is forward relative to robot
                            strafe[0] = -event.value;
                            break;
                        case AXIS_RIGHT_JOYSTICK_VERTICAL: headPitch = static_cast<float>(-event.value); break;
                        case AXIS_RIGHT_JOYSTICK_HORIZONTAL: headYaw = static_cast<float>(-event.value); break;
                    }
                }
                else if (event.isButton()) {
                    // event was a button event
                    switch (event.number) {
                        case BUTTON_TRIANGLE:
                            if (event.value > 0) {  // button down
                                if (moving) {
                                    NUClear::log("Stop walking");
                                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3f::Zero()), 2);
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
                                    NUClear::log("Head unlocked");
                                }
                                else {
                                    NUClear::log("Head locked");
                                }
                                headLocked = !headLocked;
                            }
                            break;
                            // case BUTTON_L1:
                            //     if (event.value > 0) {  // button down
                            //         NUClear::log("Requesting Left Front Kick");
                            //         emit(std::make_unique<KickScriptCommand>(KickScriptCommand(
                            //             Eigen::Vector3d::UnitX(),  // vector pointing forward relative to
                            //             robot LimbID::LEFT_LEG)));
                            //     }
                            //     break;
                            // case BUTTON_R1:
                            //     if (event.value > 0) {  // button down
                            //         NUClear::log("Requesting Right Front Kick");
                            //         emit(std::make_unique<KickScriptCommand>(KickScriptCommand(KickScriptCommand(
                            //             Eigen::Vector3d::UnitX(),  // vector pointing forward relative to
                            //             robot LimbID::RIGHT_LEG))));
                            //     }
                            //     break;
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
        // TODO(HardwareTeam): potential performance gain: ignore if value hasn't changed since last emit?
        on<Every<20, Per<std::chrono::seconds>>>().then([this] {
            if (!headLocked) {
                // Create a unit vector in the direction the head should be pointing
                Eigen::Vector3d uPCt = (Eigen::AngleAxisd(headYaw, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(-headPitch, Eigen::Vector3d::UnitY())).toRotationMatrix()* Eigen::Vector3d::UnitX();
                emit<Task>(std::make_unique<Look>(uPCt, false));
            }

            if (moving) {
                //not sure what to do here because of the math involved. might need some guidence here
                const auto strafeNorm          = strafe / std::numeric_limits<short>::max();
                const auto rotationalSpeedNorm = rotationalSpeed / std::numeric_limits<short>::max();
                Eigen::Isometry2d affineParameter;
                affineParameter.linear()      = Eigen::Rotation2Dd(rotationalSpeedNorm).toRotationMatrix();
                affineParameter.translation() = Eigen::Vector2d(strafeNorm.x(), strafeNorm.y());
                emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(affineParameter)));
            }
        });
    }
}  // namespace module::behaviour::strategy
