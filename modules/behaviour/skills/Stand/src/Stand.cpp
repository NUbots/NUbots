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

#include "Stand.h"

#include "messages/input/ServoID.h"
#include "messages/motion/Script.h"
#include "messages/behaviour/Action.h"

namespace modules {
    namespace behaviour {
        namespace skills {

            using messages::input::ServoID;
            using messages::motion::ExecuteScriptByName;
            using messages::behaviour::RegisterAction;
            using messages::behaviour::LimbID;

            //internal only callback messages to start and stop our action
            struct ExecuteStand {};

            Stand::Stand(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

                on<Trigger<ExecuteStand>>([this] (const ExecuteStand&) {

                    emit(std::make_unique<ExecuteScriptByName>(id, "Stand.yaml"));

                });

                emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
                    id,
                    { std::pair<float, std::set<LimbID>>(std::numeric_limits<float>::epsilon(), { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM}) },
                    [this] (const std::set<LimbID>&) {
                        emit(std::make_unique<ExecuteStand>());
                    },
                    [this] (const std::set<LimbID>&) { },
                    [this] (const std::set<ServoID>&) { }
                }));
            }
        }  // tools
    }  // behaviours
}  // modules
