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

#include "DirectWalkController.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/MotionCommand.hpp"
#include "message/motion/WalkCommand.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"

namespace module::behaviour::skills {

    using extension::Configuration;

    using message::behaviour::MotionCommand;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;

    DirectWalkController::DirectWalkController(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {

        // do a little configurating
        on<Configuration>("DirectWalkController.yaml").then([this](const Configuration& config) {
            log_level                = config["log_level"].as<NUClear::LogLevel>();
            cfg.direct_walk_priority = config["direct_walk_priority"].as<double>();
        });

        // Register this module with the subsumption system:
        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
            RegisterAction{subsumption_id,
                           "DirectWalkController",
                           {
                               // Limb sets required by the walk engine:
                               std::pair<double, std::set<LimbID>>(
                                   0,
                                   {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
                           },
                           [this](const std::set<LimbID>& givenLimbs) {
                               if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                                   // Enable the walk engine.
                                   emit<Scope::DIRECT>(std::make_unique<EnableWalkEngineCommand>(subsumption_id));
                               }
                           },
                           [this](const std::set<LimbID>& takenLimbs) {
                               if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                                   // Shut down the walk engine, since we don't need it right now.
                                   emit<Scope::DIRECT>(std::make_unique<DisableWalkEngineCommand>(subsumption_id));
                               }
                           },
                           [](const std::set<ServoID>& /*unused*/) {
                               // nothing
                           }}));

        on<Trigger<MotionCommand>>().then([this](const MotionCommand& command) {
            if (command.type == MotionCommand::Type::DIRECT_COMMAND) {
                update_priority(cfg.direct_walk_priority);
                emit(std::make_unique<WalkCommand>(subsumption_id, command.walk_command));
            }
            else if (command.type == MotionCommand::Type::STAND_STILL) {
                update_priority(cfg.direct_walk_priority);
                emit(std::make_unique<StopCommand>(subsumption_id));
            }
            else {
                update_priority(0);
            }
        });
    }

    void DirectWalkController::update_priority(const double& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumption_id, {priority}}));
    }
}  // namespace module::behaviour::skills
