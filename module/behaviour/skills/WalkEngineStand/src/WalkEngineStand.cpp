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

#include "extension/Configuration.h"
#include "message/motion/WalkCommand.h"
#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"

namespace module {
namespace behaviour {
    namespace skills {

        using extension::Configuration;

        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        using message::motion::DisableWalkEngineCommand;
        using message::motion::EnableWalkEngineCommand;
        using message::motion::StopCommand;
        using message::motion::WalkCommand;
        using message::motion::WalkStopped;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;

        // internal only callback messages to start and stop our action
        // struct ExecuteStand {};

        WalkEngineStand::WalkEngineStand(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
                subsumptionId,
                "WalkEngineStand",
                {std::pair<float, std::set<LimbID>>(
                    std::numeric_limits<float>::epsilon(),
                    {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
                [this](const std::set<LimbID>&) {
                    emit<Scope::DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                    emit(std::move(std::make_unique<StopCommand>(subsumptionId)));
                },
                [this](const std::set<LimbID>&) {
                    emit<Scope::DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
                },
                [this](const std::set<ServoID>&) {}}));
        }
    }  // namespace skills
}  // namespace behaviour
}  // namespace module
