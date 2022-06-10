/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "Getup.hpp"

#include <Eigen/Core>
#include <cmath>

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/behaviour/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/motion/GetupCommand.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"

namespace module::behaviour::skills {

    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using message::input::Sensors;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    Getup::Getup(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this))
        , isFront(true)
        , gettingUp(false)
        , FALLEN_ANGLE(M_PI_2) {
        // do a little configurating
        on<Configuration>("Getup.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            FALLEN_ANGLE = config["FALLEN_ANGLE"].as<float>();

            // load priorities for the getup
            GETUP_PRIORITY     = config["GETUP_PRIORITY"].as<float>();
            EXECUTION_PRIORITY = config["EXECUTION_PRIORITY"].as<float>();
            log<NUClear::WARN>("CONFIGURED");
        });

        on<Last<250, Trigger<Sensors>>, Single>().then(
            "Getup Fallen Check",
            [this](const std::list<std::shared_ptr<const Sensors>>& sensors) {
                // Transform to torso {t} from world {w} space
                Eigen::Affine3d Hwt;
                // Basis Z vector of torso {t} in world {w} space
                Eigen::Vector3d uZTw = Eigen::Vector3d::Zero();
                // Basis X vector of torso {t} in world {w} space
                Eigen::Vector3d uXTw = Eigen::Vector3d::Zero();

                // Average of the last 250 sensors measurments
                for (const auto& s : sensors) {
                    Hwt = s->Htw.inverse().matrix().cast<double>();
                    uZTw += Hwt.rotation().block(0, 2, 3, 1);
                    uXTw += Hwt.rotation().block(0, 0, 3, 1);
                }
                uZTw = uZTw.normalized();
                uXTw = uXTw.normalized();

                // Check if angle between torso and world z axis is greater than config value FALLEN_ANGLE
                if (!gettingUp && std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) > FALLEN_ANGLE) {
                    // If the z component of the torso's x basis in world space is negative, the robot is fallen on
                    // its front
                    isFront = (uXTw.z() <= 0);
                    updatePriority(GETUP_PRIORITY);
                }
            });

        on<Trigger<ExecuteGetup>, Single>().then("Execute Getup", [this]() {
            gettingUp = true;

            // Check with side we're getting up from
            if (isFront) {
                emit(std::make_unique<ExecuteScriptByName>(
                    id,
                    std::vector<std::string>({"StandUpFront.yaml", "Stand.yaml"})));
            }
            else {
                emit(std::make_unique<ExecuteScriptByName>(
                    id,
                    std::vector<std::string>({"StandUpBack.yaml", "Stand.yaml"})));
            }
            updatePriority(EXECUTION_PRIORITY);
        });

        on<Trigger<KillGetup>>().then([this] {
            gettingUp = false;
            updatePriority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            id,
            "Get Up",
            {std::pair<float, std::set<LimbID>>(
                0,
                {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
            [this](const std::set<LimbID>& /* limbs */) { emit(std::make_unique<ExecuteGetup>()); },
            [this](const std::set<LimbID>& /* limbs */) { emit(std::make_unique<KillGetup>()); },
            [this](const std::set<ServoID>& /* servos */) { emit(std::make_unique<KillGetup>()); }}));
    }

    void Getup::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{id, {priority}}));
    }

}  // namespace module::behaviour::skills
