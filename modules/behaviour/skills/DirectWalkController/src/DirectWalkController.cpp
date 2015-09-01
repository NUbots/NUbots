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

#include "DirectWalkController.h"

#include "messages/support/Configuration.h"
#include "messages/behaviour/Action.h"
#include "messages/behaviour/MotionCommand.h"
#include "messages/motion/WalkCommand.h"
#include "messages/input/LimbID.h"
#include "messages/input/ServoID.h"

namespace modules {
namespace behaviour {
namespace skills {

    using messages::support::Configuration;
    using messages::behaviour::MotionCommand;
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

    DirectWalkController::DirectWalkController(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

        // Register this module with the subsumption system:
        emit<INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            subsumptionId,
            "DirectWalkController",
            {
                // Limb sets required by the walk engine:
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
            },
            [this] (const std::set<LimbID>& givenLimbs) {
                if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                    // Enable the walk engine.
                    emit<DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                }
            },
            [this] (const std::set<LimbID>& takenLimbs) {
                if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                    // Shut down the walk engine, since we don't need it right now.
                    emit<DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
                }
            },
            [this] (const std::set<ServoID>&) {
                // nothing
            }
        }));

        on<Trigger<MotionCommand>>().then([this] (const MotionCommand& command) {
            if (command.type == MotionCommand::Type::DirectCommand) {
                emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 26, 11 }}));
                emit(std::move(std::make_unique<WalkStartCommand>(subsumptionId)));
                emit(std::move(std::make_unique<WalkCommand>(subsumptionId, command.walkCommand)));
            } else if (command.type == MotionCommand::Type::StandStill) {
                emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 26, 11 }}));
                emit(std::move(std::make_unique<WalkStopCommand>(subsumptionId)));
            } else {
                emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 0, 0 }}));
            }
        });

        on<Trigger<WalkStopped>>().then([this] {
            emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 0, 0 }}));
        });
    }
}
}
}
