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

#include "WalkEngineStand.h"

#include "messages/support/Configuration.h"
#include "messages/behaviour/Action.h"
#include "messages/motion/WalkCommand.h"
#include "messages/input/LimbID.h"
#include "messages/input/ServoID.h"

namespace modules {
namespace behaviour {
namespace skills {

    using messages::support::Configuration;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;
    using messages::motion::WalkStopped;
    using messages::motion::WalkCommand;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;
    using messages::motion::EnableWalkEngineCommand;
    using messages::motion::DisableWalkEngineCommand;
    using messages::input::LimbID;
    using messages::input::ServoID;

	//internal only callback messages to start and stop our action
    // struct ExecuteStand {};

    WalkEngineStand::WalkEngineStand(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

		emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            subsumptionId,
            "WalkEngineStand",
            { std::pair<float, std::set<LimbID>>(std::numeric_limits<float>::epsilon(), { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM}) },
            [this] (const std::set<LimbID>&) {
                emit<Scope::DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                emit(std::move(std::make_unique<WalkStopCommand>(subsumptionId)));
            },
            [this] (const std::set<LimbID>&) {
                emit<Scope::DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
            },
            [this] (const std::set<ServoID>&) { }
        }));

        on<Trigger<Configuration<WalkEngineStand>>>([this] (const Configuration<WalkEngineStand>& config) {
            // Use configuration here from file WalkEngineStand.yaml
        });

        // on<Trigger<ExecuteStand>>([this] (const ExecuteStand&) {

        // });
    }
}
}
}
