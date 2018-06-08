/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "Jump.h"

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/behaviour/ServoCommand.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"

namespace module {
namespace behaviour {
    namespace tools {

        using extension::Configuration;
        using extension::ExecuteScriptByName;

        using message::platform::darwin::ButtonLeftDown;
        using message::platform::darwin::ButtonMiddleDown;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        Jump::Jump(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{2,
                               "Jump",
                               {std::pair<float, std::set<LimbID>>(
                                   100, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<ServoID>&) {}}));

            on<Trigger<ButtonMiddleDown>>().then([this] {
                std::this_thread::sleep_for(std::chrono::seconds(2));

                emit(std::make_unique<ExecuteScriptByName>(2, std::vector<std::string>({"Jump.yaml"})));
            });
        }
    }  // namespace tools
}  // namespace behaviour
}  // namespace module
