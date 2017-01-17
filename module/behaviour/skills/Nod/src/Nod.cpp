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

#include "Nod.h"
#include <nuclear>

#include "message/input/ServoID.h"
#include "message/motion/Script.h"
#include "message/behaviour/Action.h"
#include "extension/Configuration.h"
#include "message/behaviour/Nod.h"

namespace module {
namespace behaviour {
namespace skills {

    struct ExecuteNod {};

    using extension::Configuration;
    using message::input::ServoID;
    using message::motion::ExecuteScriptByName;
    using message::behaviour::RegisterAction;
    using message::behaviour::ActionPriorites;
    using message::input::LimbID;

    Nod::Nod(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this))
        , value(false)
        , EXECUTION_PRIORITY(0.0f) {

        // do a little configurating
        on<Configuration>("Nod.yaml").then([this] (const Configuration& config) {
            EXECUTION_PRIORITY = config["execution_priority"].as<float>();
        });

        on<Trigger<message::behaviour::Nod>>().then([this] (const message::behaviour::Nod& nod) {
            value = nod.value;
            updatePriority(EXECUTION_PRIORITY);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "Nod",
            { std::pair<float, std::set<LimbID>>(0, { LimbID::HEAD }) },
            [this] (const std::set<LimbID>&) {
                if (value) {
                    emit(std::make_unique<ExecuteScriptByName>(id,  std::vector<std::string>({"NodYes.yaml"})));
                }
                else {
                    emit(std::make_unique<ExecuteScriptByName>(id,  std::vector<std::string>({"NodNo.yaml"})));
                }
            },
            [this] (const std::set<LimbID>&) {
                updatePriority(0);
            },
            [this] (const std::set<ServoID>&) {
                updatePriority(0);
            }
        }));
    }

    void Nod::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { priority }}));
    }

}
}
}

