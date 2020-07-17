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

#include "extension/Configuration.h"

#include "message/behaviour/MotionCommand.h"
#include "message/motion/WalkCommand.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"

namespace module {
namespace behaviour {
    namespace skills {

        using extension::Configuration;

        using message::behaviour::MotionCommand;
        using message::motion::DisableWalkEngineCommand;
        using message::motion::EnableWalkEngineCommand;
        using message::motion::StopCommand;
        using message::motion::WalkCommand;
        using message::motion::WalkStopped;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;

        DirectWalkController::DirectWalkController(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

            // Register this module with the subsumption system:
            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
                subsumptionId,
                "DirectWalkController",
                {
                    // Limb sets required by the walk engine:
                    std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                    std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
                },
                [this](const std::set<LimbID>& givenLimbs) {
                    if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                        // Enable the walk engine.
                        emit<Scope::DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                    }
                },
                [this](const std::set<LimbID>& takenLimbs) {
                    if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                        // Shut down the walk engine, since we don't need it right now.
                        emit<Scope::DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
                    }
                },
                [this](const std::set<ServoID>&) {
                    // nothing
                }}));

            on<Trigger<MotionCommand>>().then([this](const MotionCommand& command) {
                if (command.type == MotionCommand::Type::DirectCommand) {
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {26, 11}}));
                    emit(std::move(std::make_unique<WalkCommand>(subsumptionId, command.walkCommand)));
                }
                else if (command.type == MotionCommand::Type::StandStill) {
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {26, 11}}));
                    emit(std::move(std::make_unique<StopCommand>(subsumptionId)));
                }
                else {
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {0, 0}}));
                }
            });

            on<Trigger<WalkStopped>>().then([this] {
                // TODO : Right now, this causes the walk engine to become unaware of positions and negatively impact
                // servo positions...
                // emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 0, 0 }}));
            });
        }
    }  // namespace skills
}  // namespace behaviour
}  // namespace module
