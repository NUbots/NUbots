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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "FallingRelax.h"

#include <cmath>
#include "messages/input/ServoID.h"
#include "messages/motion/Script.h"
#include "messages/behaviour/Action.h"
#include "messages/behaviour/ServoCommand.h"
#include "messages/support/Configuration.h"
#include "messages/input/Sensors.h"

namespace modules {
    namespace behaviour {
        namespace skills {

            //internal only callback messages to start and stop our action
            struct Falling {};
            struct KillFalling {};

            using messages::support::Configuration;
            using messages::input::Sensors;
            using messages::input::ServoID;
            using messages::motion::ExecuteScriptByName;
            using messages::behaviour::RegisterAction;
            using messages::behaviour::ActionPriorites;
            using messages::input::LimbID;

            FallingRelax::FallingRelax(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)), falling(false) {

                //do a little configurating
                on<Trigger<Configuration<FallingRelax>>>([this] (const Configuration<FallingRelax>& config){

                    // Store falling angle as a cosine so we can compare it directly to the z axis value
                    double fallingAngle = config["FALLING_ANGLE"].as<double>();
                    FALLING_ANGLE = cos(fallingAngle);

                    // When falling the acceleration should drop below this value
                    FALLING_ACCELERATION = config["FALLING_ACCELERATION"].as<float>();

                    // Once the acceleration has stabalized, we are no longer falling
                    RECOVERY_ACCELERATION = config["RECOVERY_ACCELERATION"].as<std::vector<float>>();

                    PRIORITY = config["PRIORITY"].as<float>();
                });

                on<Trigger<Last<5, Sensors>>, Options<Single>>([this] (const LastList<Sensors>& sensors) {

                    if(!falling
                        && !sensors.empty()
                        && fabs(sensors.back()->orientation(2,2)) < FALLING_ANGLE) {

                        // We might be falling, check the accelerometer
                        double magnitude = 0;

                        for(const auto& sensor : sensors) {
                            magnitude += arma::norm(sensor->accelerometer, 2);
                        }

                        magnitude /= sensors.size();

                        if(magnitude < FALLING_ACCELERATION) {
                            falling = true;
                            updatePriority(PRIORITY);
                        }

                    }
                    else if(falling) {
                        // We might be recovered, check the accelerometer
                        double magnitude = 0;

                        for(const auto& sensor : sensors) {
                            magnitude += arma::norm(sensor->accelerometer, 2);
                        }

                        magnitude /= sensors.size();

                        // See if we recover
                        if(magnitude > RECOVERY_ACCELERATION[0] && magnitude < RECOVERY_ACCELERATION[1]) {
                            falling = false;
                            updatePriority(0);
                        }
                    }
                });

                on<Trigger<Falling>>([this] (const Falling&) {

                    emit(std::make_unique<ExecuteScriptByName>(id, "Relax.yaml"));
                });

                on<Trigger<KillFalling>>([this] (const KillFalling&) {
                    falling = false;
                    updatePriority(0);
                });

                emit<INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
                    id,
                    "Falling Relax",
                    { std::pair<float, std::set<LimbID>>(0, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD }) },
                    [this] (const std::set<LimbID>&) {
                        emit(std::make_unique<Falling>());
                    },
                    [this] (const std::set<LimbID>&) {
                        emit(std::make_unique<KillFalling>());
                    },
                    [this] (const std::set<ServoID>&) {
                        // Ignore
                    }
                }));
            }

            void FallingRelax::updatePriority(const float& priority) {
                emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { priority }}));
            }

        }  // skills
    }  // behaviours
}  // modules
