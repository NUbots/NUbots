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

#include "Look.h"

#include "messages/input/ServoID.h"
#include "messages/behaviour/LookStrategy.h"
#include "messages/behaviour/Action.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"

namespace modules {
    namespace behaviour {
        namespace skills {

            using messages::input::ServoID;
            using messages::input::Sensors;
            using messages::behaviour::Look;
            using messages::behaviour::LimbID;
            using messages::support::Configuration;
            using messages::behaviour::ServoCommand;

            //internal only callback messages to start and stop our action
            struct ExecuteLook {};

            LookAt::LookAt(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

                //do a little configurating
                on<Trigger<Configuration<Look>>>([this] (const Configuration<Look>& config){

                    lastPanEnd = NUClear::clock::now();
                    //load fast and slow panspeed settings

                    //pan speeds
                    FAST_SPEED = config["speed"]["fast"].as<double>();
                    SLOW_SPEED = config["speed"]["slow"].as<double>();

                    HEAD_PITCH_MIN = config["limits"]["pitch"][0].as<double>();
                    HEAD_PITCH_MAX = config["limits"]["pitch"][1].as<double>();
                    HEAD_YAW_MAX   = config["limits"]["yaw"][0].as<double>();
                    HEAD_YAW_MIN   = config["limits"]["yaw"][1].as<double>();
                    SCREEN_EDGE_PADDING = config["screen_edge_padding"].as<double>();
                });

                on<Trigger<std::vector<Look::Fixation>>>([this](const std::vector<Look::Fixation>& fixation) {

                });

                on<Trigger<std::vector<Look::Pan>>>([this](const std::vector<Look::Pan>& pan) {

                });

                on<Trigger<std::vector<Look::Saccade>>>([this](const std::vector<Look::Saccade>& saccade) {

                });

                emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
                    id,
                    "Look",
                    { std::pair<float, std::set<LimbID>>(30.0, { LimbID::HEAD }) },
                    [this] (const std::set<LimbID>&) {
                        emit(std::make_unique<ExecuteLook>());
                    },
                    [this] (const std::set<LimbID>&) { },
                    [this] (const std::set<ServoID>&) { }
                }));
            }
        }  // reflexes
    }  // behaviours
}  // modules
