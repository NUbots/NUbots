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

#include "FallingRelax.hpp"

#include <cmath>

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/behaviour/ServoCommand.hpp"
#include "message/input/Sensors.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"

namespace module::behaviour::skills {

    // internal only callback messages to start and stop our action
    struct Falling {};
    struct KillFalling {};

    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using message::input::Sensors;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;
    using utility::input::LimbID;
    using utility::input::ServoID;

    FallingRelax::FallingRelax(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // do a little configurating
        on<Configuration>("FallingRelax.yaml").then([this](const Configuration& config) {
            // Store falling angle as a cosine so we can compare it directly to the z axis value
            const auto fallingAngle = config["FALLING_ANGLE"].as<double>();
            FALLING_ANGLE           = std::cos(fallingAngle);

            // When falling the acceleration should drop below this value
            FALLING_ACCELERATION = config["FALLING_ACCELERATION"].as<double>();

            // Once the acceleration has stabalized, we are no longer falling
            RECOVERY_ACCELERATION = config["RECOVERY_ACCELERATION"].as<std::vector<double>>();

            PRIORITY = config["PRIORITY"].as<double>();
        });

        on<Last<5, Trigger<Sensors>>, Single>([this](const std::list<std::shared_ptr<const Sensors>>& sensors) {
            if (!falling && !sensors.empty() && fabs(sensors.back()->Htw(2, 2)) < FALLING_ANGLE) {

                // We might be falling, check the accelerometer
                double magnitude = 0;

                for (const auto& sensor : sensors) {
                    magnitude += sensor->accelerometer.norm();
                }

                magnitude /= sensors.size();

                if (magnitude < FALLING_ACCELERATION) {
                    falling = true;
                    updatePriority(PRIORITY);
                }
            }
            else if (falling) {
                // We might be recovered, check the accelerometer
                double magnitude = 0;

                for (const auto& sensor : sensors) {
                    magnitude += sensor->accelerometer.norm();
                }

                magnitude /= sensors.size();

                // See if we recover
                if (magnitude > RECOVERY_ACCELERATION[0] && magnitude < RECOVERY_ACCELERATION[1]) {
                    falling = false;
                    updatePriority(0);
                }
            }
        });

        on<Trigger<Falling>>().then([this] { emit(std::make_unique<ExecuteScriptByName>(id, "Relax.yaml")); });

        on<Trigger<KillFalling>>().then([this] {
            falling = false;
            updatePriority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            id,
            "Falling Relax",
            {std::pair<double, std::set<LimbID>>(
                0.0f,
                {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
            [this](const std::set<LimbID>& /*unused*/) { emit(std::make_unique<Falling>()); },
            [this](const std::set<LimbID>& /*unused*/) { emit(std::make_unique<KillFalling>()); },
            [](const std::set<ServoID>& /*unused*/) {
                // Ignore
            }}));
    }

    void FallingRelax::updatePriority(const double& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{id, {priority}}));
    }

}  // namespace module::behaviour::skills
