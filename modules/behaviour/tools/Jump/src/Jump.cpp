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

#include "messages/support/Configuration.h"


#include "messages/input/ServoID.h"
#include "messages/motion/Script.h"
#include "messages/behaviour/Action.h"
#include "messages/behaviour/ServoCommand.h"
#include "messages/input/Sensors.h"
#include "messages/platform/darwin/DarwinSensors.h"

namespace modules {
namespace behaviour {
namespace tools {

    using messages::support::Configuration;
    using messages::support::Configuration;
    using messages::input::ServoID;
    using messages::motion::ExecuteScriptByName;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;
    using messages::input::LimbID;
    using messages::platform::darwin::ButtonMiddleDown;
    using messages::platform::darwin::ButtonLeftDown;

    Jump::Jump(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            2,
            "Jump",
            { std::pair<float, std::set<LimbID>>(100, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM }) },
            [this] (const std::set<LimbID>&) {
            },
            [this] (const std::set<LimbID>&) {
            },
            [this] (const std::set<ServoID>&) {
            }
        }));

        on<Trigger<ButtonMiddleDown>>().then([this] {
            std::this_thread::sleep_for(std::chrono::seconds(2));

            emit(std::make_unique<ExecuteScriptByName>(2,  std::vector<std::string>({"Jump.yaml"})));
        });
    }
}
}
}
