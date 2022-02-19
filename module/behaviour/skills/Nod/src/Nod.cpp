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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#include "Nod.hpp"

#include <nuclear>

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/behaviour/Nod.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"

namespace module::behaviour::skills {

    struct ExecuteNod {};

    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;

    Nod::Nod(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // do a little configurating
        on<Configuration>("Nod.yaml").then([this](const Configuration& config) {
            EXECUTION_PRIORITY = config["execution_priority"].as<float>();
        });

        on<Trigger<message::behaviour::Nod>>().then([this](const message::behaviour::Nod& nod) {
            value = nod.value;
            updatePriority(EXECUTION_PRIORITY);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            id,
            "Nod",
            {std::pair<float, std::set<LimbID>>(0, {LimbID::HEAD})},
            [this](const std::set<LimbID>& /*unused*/) {
                if (value) {
                    emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"NodYes.yaml"})));
                }
                else {
                    emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"NodNo.yaml"})));
                }
            },
            [this](const std::set<LimbID>& /*unused*/) { updatePriority(0); },
            [this](const std::set<ServoID>& /*unused*/) { updatePriority(0); }}));
    }

    void Nod::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{id, {priority}}));
    }
}  // namespace module::behaviour::skills
