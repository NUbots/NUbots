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
#include <Eigen/Geometry>
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
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {
        // do a little configurating
        on<Configuration>("Getup.yaml").then([this](const Configuration& config) {
            log_level          = config["log_level"].as<NUClear::LogLevel>();
            cfg.fallen_angle   = config["fallen_angle"].as<float>();
            cfg.getup_priority = config["getup_priority"].as<float>();

            cfg.getup_front = config["getup_front"].as<std::vector<std::string>>();
            cfg.getup_back  = config["getup_back"].as<std::vector<std::string>>();
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            subsumption_id,
            "Get Up",
            {std::pair<float, std::set<LimbID>>(
                0,
                {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
            [this](const std::set<LimbID>& /* limbs */) { emit(std::make_unique<ExecuteGetup>()); },
            [this](const std::set<LimbID>& /* limbs */) { emit(std::make_unique<KillGetup>()); },
            [this](const std::set<ServoID>& /* servos */) { emit(std::make_unique<KillGetup>()); }}));

        on<Trigger<Sensors>>().then("Getup Fallen Check", [this](const Sensors& sensors) {
            // Transform to torso {t} from world {w} space
            Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
            // Basis Z vector of torso {t} in world {w} space
            Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);
            // Basis X vector of torso {t} in world {w} space
            Eigen::Vector3d uXTw = Hwt.block(0, 0, 3, 1);

            // Check if angle between torso z axis and world z axis is greater than config value cfg.fallen_angle
            if (!getting_up && std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) > cfg.fallen_angle) {
                // If the z component of the torso's x basis in world space is negative, the robot is fallen on
                // its front
                is_front = (uXTw.z() <= 0);
                update_priority(cfg.getup_priority);
            }
        });

        on<Trigger<ExecuteGetup>, Single>().then("Execute Getup", [this]() {
            getting_up = true;

            // Check with side we're getting up from
            if (is_front) {
                emit(std::make_unique<ExecuteScriptByName>(subsumption_id, std::vector<std::string>(cfg.getup_front)));
            }
            else {
                emit(std::make_unique<ExecuteScriptByName>(subsumption_id, std::vector<std::string>(cfg.getup_back)));
            }
        });

        on<Trigger<KillGetup>>().then([this] {
            getting_up = false;
            update_priority(0);
        });
    }

    void Getup::update_priority(const float& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumption_id, {priority}}));
    }

}  // namespace module::behaviour::skills
