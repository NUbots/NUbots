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

#include <fmt/format.h>

#include "PS3Walk.h"

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/behaviour/MotionCommand.h"
#include "message/behaviour/ServoCommand.h"
#include "message/input/PS3Controller.h"
#include "message/motion/HeadCommand.h"
#include "message/motion/KickCommand.h"

#include "utility/behaviour/Action.h"
#include "utility/behaviour/MotionCommand.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace behaviour {
    namespace strategy {

        using extension::Configuration;
        using extension::ExecuteScriptByName;

        using message::behaviour::MotionCommand;
        using message::input::PS3Controller::CircleButton;
        using message::input::PS3Controller::CrossButton;
        using message::input::PS3Controller::L1Button;
        using message::input::PS3Controller::LeftJoystick;
        using message::input::PS3Controller::R1Button;
        using message::input::PS3Controller::RightJoystick;
        using message::input::PS3Controller::SquareButton;
        using message::input::PS3Controller::TriangleButton;
        using message::motion::HeadCommand;
        using message::motion::KickScriptCommand;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::math::matrix::Transform2D;

        PS3Walk::PS3Walk(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

            on<Configuration>("PS3Walk.yaml").then([this](const Configuration& config) {
                actions.clear();
                for (const auto& action : config["action_scripts"].as<std::vector<std::string>>()) {
                    actions.push_back(action);
                }
            });

            on<Trigger<TriangleButton>>().then([this](const TriangleButton& button) {
                if (button.pressed) {
                    if (moving) {
                        NUClear::log("Stop walking");
                        emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                    }
                    else {
                        NUClear::log("Start walking");
                    }
                    moving = !moving;
                }
            });

            on<Trigger<SquareButton>>().then([this](const SquareButton& button) {
                if (button.pressed) {
                    if (headLocked) {
                        NUClear::log("Head unlocked");
                    }
                    else {
                        NUClear::log("Head locked");
                    }
                    headLocked = !headLocked;
                }
            });

            on<Trigger<CrossButton>>().then([this](const CrossButton& button) {
                if (button.pressed) {
                    NUClear::log("Triggering actions");
                    emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {90}}));
                    emit(std::make_unique<ExecuteScriptByName>(id, actions));
                }
            });

            on<Trigger<CircleButton>>().then([this](const CircleButton& button) {
                if (button.pressed) {
                    NUClear::log("Standing");
                    emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {90}}));
                    emit(std::make_unique<ExecuteScriptByName>(id, "Stand.yaml"));
                }
            });

            on<Trigger<L1Button>>().then([this](const L1Button& button) {
                if (button.pressed) {
                    NUClear::log("Requesting Left Front Kick");
                    emit(std::make_unique<KickScriptCommand>(
                        KickScriptCommand{Eigen::Vector3d(1, 0, 0),  // vector pointing forward relative to robot
                                          LimbID::LEFT_LEG}));
                }
            });

            on<Trigger<R1Button>>().then([this](const R1Button& button) {
                if (button.pressed) {
                    NUClear::log("Requesting Right Front Kick");
                    emit(std::make_unique<KickScriptCommand>(
                        KickScriptCommand(Eigen::Vector3d(1, 0, 0),  // vector pointing forward relative to robot
                                          LimbID::RIGHT_LEG)));
                }
            });

            on<Trigger<LeftJoystick>>().then([this](const LeftJoystick& joystick) {
                switch (joystick.direction.value) {
                    case LeftJoystick::Direction::HORIZONTAL:
                        // y is left relative to robot
                        // strafe[1] = -joystick.value;
                        rotationalSpeed = -joystick.value;
                        break;
                    case LeftJoystick::Direction::VERTICAL:
                        // x is forward relative to robot
                        strafe[0] = -joystick.value;
                        break;
                }
            });

            on<Trigger<RightJoystick>>().then([this](const RightJoystick& joystick) {
                switch (joystick.direction.value) {
                    case RightJoystick::Direction::HORIZONTAL: headYaw = -joystick.value; break;
                    case RightJoystick::Direction::VERTICAL: headPitch = -joystick.value; break;
                }
            });

            // output walk command based on updated strafe and rotation speed from joystick
            // TODO: potential performance gain: ignore if value hasn't changed since last emit?
            on<Every<20, Per<std::chrono::seconds>>>().then([this] {
                if (!headLocked) {
                    auto headCommand        = std::make_unique<HeadCommand>();
                    headCommand->yaw        = headYaw * 1.5;
                    headCommand->pitch      = headPitch;
                    headCommand->robotSpace = true;
                    emit(std::move(headCommand));
                }

                if (moving) {
                    // TODO: hacked to not allow backwards movement for stability
                    // arma::vec s = { std::max(strafe[0], 0.0), strafe[1] };
                        auto transform = Transform2D(strafe * max_speed, rotationalSpeed * max_rotational_speed);
                        emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(transform)));
                }
            });

            emit<Scope::DIRECT>(std::make_unique<RegisterAction>(RegisterAction{
                id,
                "PS3Walk",
                {std::pair<float, std::set<LimbID>>(
                    0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
                [this](const std::set<LimbID>&) {},
                [this](const std::set<LimbID>&) {},
                [this](const std::set<ServoID>&) {
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {0}}));
                }}));
        }
    }  // namespace strategy
}  // namespace behaviour
}  // namespace module
