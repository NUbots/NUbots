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

#include "Stand.h"

#include "extension/Script.h"
#include "message/behaviour/ServoCommand.h"
#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"

namespace module {
namespace behaviour {
    namespace skills {

        using extension::ExecuteScriptByName;

        using utility::behaviour::RegisterAction;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        // internal only callback messages to start and stop our action
        struct ExecuteStand {};

        Stand::Stand(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

            on<Trigger<ExecuteStand>>().then([this] { emit(std::make_unique<ExecuteScriptByName>(id, "Stand.yaml")); });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{id,
                               "Stand",
                               {std::pair<float, std::set<LimbID>>(
                                   std::numeric_limits<float>::epsilon(),
                                   {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
                               [this](const std::set<LimbID>&) { emit(std::make_unique<ExecuteStand>()); },
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<ServoID>&) {}}));
        }
    }  // namespace skills
}  // namespace behaviour
}  // namespace module
