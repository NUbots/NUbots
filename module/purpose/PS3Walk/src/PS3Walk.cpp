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

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/input/LimbID.hpp"

namespace module::purpose {

    using extension::Configuration;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::skill::Kick;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;
    using utility::input::LimbID;


    PS3Walk::PS3Walk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without these emis, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));

            // On the lowest level, just stand
            emit<Task>(std::make_unique<StandStill>(), 0);
            // On the highest level, recover from falling
            emit<Task>(std::make_unique<FallRecovery>(), 3);
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
                                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 1);
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
                            //         emit<Task>(std::make_unique<Kick>(LimbID::LEFT_LEG), 2);
                            //     }
                            //     break;
                            // case BUTTON_R1:
                            //     if (event.value > 0) {  // button down
                            //         NUClear::log("Requesting Right Front Kick");
                            //         emit<Task>(std::make_unique<Kick>(LimbID::RIGHT_LEG), 2);
                            //     }
                            //     break;
                            // case BUTTON_L2:
                            //     if (event.value > 0) {  // button down
                            //         NUClear::log("Requesting Left Side Kick");
                            //     }
                            //     break;
                            // case BUTTON_R2:
                            //     if (event.value > 0) {  // button down
                            //         NUClear::log("Requesting Right Side Kick");
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
                double yaw           = headYaw / std::numeric_limits<short>::max() * 1.5f;
                double pitch         = headPitch / std::numeric_limits<short>::max();
                Eigen::Vector3d uPCt = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                        * Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()))
                                           .toRotationMatrix()
                                       * Eigen::Vector3d::UnitX();
                emit<Task>(std::make_unique<Look>(uPCt, false));
            }

            // If moving, then determine the walk command and request it
            if (moving) {
                const auto strafe_norm           = strafe / std::numeric_limits<short>::max();
                const auto rotational_speed_norm = rotationalSpeed / std::numeric_limits<short>::max();
                emit<Task>(
                    std::make_unique<Walk>(Eigen::Vector3d(strafe_norm[0], strafe_norm[1], rotational_speed_norm)),
                    1);
            }
        });
    }
}  // namespace module::purpose
