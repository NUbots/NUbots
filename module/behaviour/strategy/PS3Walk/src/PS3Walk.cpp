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
        using ScriptTriggerEvent = message::input::ps3controller::CircleButton;
        using TriggerActionEvent = message::input::ps3controller::CrossButton;
        using LeftKickEvent      = message::input::ps3controller::L1Button;
        using WalkVelocityEvent  = message::input::ps3controller::LeftJoystick;
        using RightKickEvent     = message::input::ps3controller::R1Button;
        using HeadDirectionEvent = message::input::ps3controller::RightJoystick;
        using HeadControlEvent   = message::input::ps3controller::SquareButton;
        using WalkControlEvent   = message::input::ps3controller::TriangleButton;
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
                walkCommandLimits << config["max_speed"]["linear"].as<float>(),
                    config["max_speed"]["linear"].as<float>(), config["max_speed"]["rotational"].as<float>();
            });

            on<Trigger<WalkControlEvent>>().then([this](const WalkControlEvent& event) {
                if (event.pressed) {
                    if (moving) {
                        log<NUClear::INFO>("Stop walking");
                        emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                        moving = false;
                    }
                    else {
                        log<NUClear::INFO>("Start walking");
                        moving = true;
                    }
                }
            });

            on<Trigger<HeadControlEvent>>().then([this](const HeadControlEvent& event) {
                if (event.pressed) {
                    if (headLocked) {
                        log<NUClear::INFO>("Head unlocked");
                        headLocked = false;
                    }
                    else {
                        log<NUClear::INFO>("Head locked");
                        headLocked = true;
                    }
                }
            });

            on<Trigger<TriggerActionEvent>>().then([this](const TriggerActionEvent& event) {
                if (event.pressed) {
                    log<NUClear::INFO>("Triggering actions");
                    emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {90}}));
                    emit(std::make_unique<ExecuteScriptByName>(id, actions));
                }
            });

            on<Trigger<ScriptTriggerEvent>>().then([this](const ScriptTriggerEvent& event) {
                if (event.pressed) {
                    log<NUClear::INFO>("Standing");
                    emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {90}}));
                    emit(std::make_unique<ExecuteScriptByName>(id, "Stand.yaml"));
                }
            });

            on<Trigger<LeftKickEvent>>().then([this](const LeftKickEvent& event) {
                if (event.pressed) {
                    log<NUClear::INFO>("Requesting Left Front Kick");
                    // vector pointing forward relative to robot
                    emit(std::make_unique<KickScriptCommand>(
                        KickScriptCommand{Eigen::Vector3d(1.0, 0.0, 0.0), LimbID::LEFT_LEG}));
                }
            });

            on<Trigger<RightKickEvent>>().then([this](const RightKickEvent& event) {
                if (event.pressed) {
                    log<NUClear::INFO>("Requesting Right Front Kick");
                    // vector pointing forward relative to robot
                    emit(std::make_unique<KickScriptCommand>(
                        KickScriptCommand{Eigen::Vector3d(1.0, 0.0, 0.0), LimbID::RIGHT_LEG}));
                }
            });

            on<Trigger<WalkVelocityEvent>>().then([this](const WalkVelocityEvent& event) {
                switch (event.direction.value) {
                    case WalkVelocityEvent::Direction::HORIZONTAL:
                        // y is left relative to robot
                        walkCommand.z() = -event.value;
                        break;
                    case WalkVelocityEvent::Direction::VERTICAL:
                        // x is forward relative to robot
                        walkCommand.x() = -event.value;
                        break;
                }
            });

            on<Trigger<HeadDirectionEvent>>().then([this](const HeadDirectionEvent& event) {
                switch (event.direction.value) {
                    case HeadDirectionEvent::Direction::HORIZONTAL: headYaw = -event.value; break;
                    case HeadDirectionEvent::Direction::VERTICAL: headPitch = -event.value; break;
                }
            });

            on<Every<20, Per<std::chrono::seconds>>>().then([this] {
                // Output head command based on updated information from joystick
                if (!headLocked) {
                    auto headCommand        = std::make_unique<HeadCommand>();
                    headCommand->yaw        = headYaw * 1.5;
                    headCommand->pitch      = headPitch;
                    headCommand->robotSpace = true;
                    emit(std::move(headCommand));
                }

                // Output walk command based on updated strafe and rotation speed from joystick
                if (moving) {
                    if (((prevWalkCommand - walkCommand).array().abs() > 0.1).any()) {
                        Eigen::Vector3d command = walkCommand.cwiseProduct(walkCommandLimits);
                        emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(command)));
                        prevWalkCommand = walkCommand;
                    }
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
