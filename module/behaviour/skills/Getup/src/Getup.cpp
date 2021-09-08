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
#include "message/motion/GetupCommand.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"

namespace module::behaviour::skills {

    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::platform::RawSensors;

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
        });

        on<Last<20, Trigger<RawSensors>>, Single>().then(
            "Getup Fallen Check",
            [this](const std::list<std::shared_ptr<const RawSensors>>& sensors) {
                Eigen::Vector3d acc_reading = Eigen::Vector3d::Zero();

                for (const auto& s : sensors) {
                    acc_reading += s->accelerometer.cast<double>();
                }
                acc_reading = (acc_reading / double(sensors.size())).normalized();

                // check that the accelerometer reading is less than some predetermined
                // amount
                if (!gettingUp && std::acos(Eigen::Vector3d::UnitZ().dot(acc_reading)) > FALLEN_ANGLE) {
                    // If we are on our side, treat it as being on our front, hopefully the rollover will help matters
                    isFront = (M_PI_2 - std::acos(Eigen::Vector3d::UnitX().dot(acc_reading)) <= 0.0);

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
