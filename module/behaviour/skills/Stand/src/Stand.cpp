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

#include "Stand.hpp"

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/behaviour/ServoCommand.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"

namespace module::behaviour::skills {

    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using utility::behaviour::RegisterAction;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    // internal only callback messages to start and stop our action
    struct ExecuteStand {};

    Stand::Stand(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {

        // do a little configurating
        on<Configuration>("Stand.yaml").then([this](const Configuration& config) {
            log_level          = config["log_level"].as<NUClear::LogLevel>();
            cfg.stand_priority = config["stand_priority"].as<double>();
        });

        on<Trigger<ExecuteStand>>().then(
            [this] { emit(std::make_unique<ExecuteScriptByName>(subsumption_id, "Stand.yaml")); });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
            RegisterAction{subsumption_id,
                           "Stand",
                           {std::pair<double, std::set<LimbID>>(
                               cfg.stand_priority,
                               {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
                           [this](const std::set<LimbID>& /*unused*/) { emit(std::make_unique<ExecuteStand>()); },
                           [](const std::set<LimbID>& /*unused*/) {},
                           [](const std::set<ServoID>& /*unused*/) {}}));
    }
}  // namespace module::behaviour::skills
