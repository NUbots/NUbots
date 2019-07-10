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

#include "FallingRelax.h"

#include <cmath>

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace behaviour {
    namespace skills {

        // internal only callback messages to start and stop our action
        struct Falling {};
        struct KillFalling {};

        using extension::Configuration;
        using extension::ExecuteScriptByName;

        using message::input::Sensors;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        FallingRelax::FallingRelax(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , id(size_t(this) * size_t(this) - size_t(this))
            , falling(false)
            , FALLING_ANGLE(0.0f)
            , FALLING_ACCELERATION(0.0f)
            , RECOVERY_ACCELERATION()
            , PRIORITY(0.0f) {

            // do a little configurating
            on<Configuration>("FallingRelax.yaml").then([this](const Configuration& config) {
                // Store falling angle as a cosine so we can compare it directly to the z axis value
                double fallingAngle = config["FALLING_ANGLE"].as<double>();
                FALLING_ANGLE       = cos(fallingAngle);

                // When falling the acceleration should drop below this value
                FALLING_ACCELERATION = config["FALLING_ACCELERATION"].as<float>();

                // Once the acceleration has stabalized, we are no longer falling
                RECOVERY_ACCELERATION = config["RECOVERY_ACCELERATION"].as<std::vector<float>>();

                PRIORITY = config["PRIORITY"].as<float>();
            });

            on<Last<5, Trigger<Sensors>>, Single>([this](const std::list<std::shared_ptr<const Sensors>>& sensors) {
                if (!falling && !sensors.empty() && fabs(sensors.back()->Htw(2, 2)) < FALLING_ANGLE) {

                    // We might be falling, check the accelerometer
                    double magnitude = 0;

                    for (const auto& sensor : sensors) {
                        magnitude += arma::norm(convert(sensor->accelerometer), 2);
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
                        magnitude += arma::norm(convert(sensor->accelerometer), 2);
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
                {std::pair<float, std::set<LimbID>>(
                    0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
                [this](const std::set<LimbID>&) { emit(std::make_unique<Falling>()); },
                [this](const std::set<LimbID>&) { emit(std::make_unique<KillFalling>()); },
                [this](const std::set<ServoID>&) {
                    // Ignore
                }}));
        }

        void FallingRelax::updatePriority(const float& priority) {
            emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {priority}}));
        }

    }  // namespace skills
}  // namespace behaviour
}  // namespace module
