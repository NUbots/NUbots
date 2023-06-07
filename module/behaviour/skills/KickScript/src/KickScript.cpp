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
#include "KickScript.hpp"

#include <nuclear>

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/behaviour/ServoCommand.hpp"
#include "message/motion/WalkCommand.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::behaviour::skills {

    struct ExecuteKick {};
    struct FinishKick {};

    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    using message::motion::KickCommandType;
    using message::motion::KickFinished;
    using message::motion::KickScriptCommand;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;

    using utility::support::Expression;

    KickScript::KickScript(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {

        // do a little configurating
        on<Configuration>("KickScript.yaml").then([this](const Configuration& config) {
            log_level           = config["log_level"].as<NUClear::LogLevel>();
            cfg.kick_priority   = config["kick_priority"].as<double>();
            cfg.message_timeout = config["message_timeout"].as<Expression>();
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            subsumption_id,
            "Kick Script",
            {std::pair<double, std::set<LimbID>>(
                0,
                {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
            [this](const std::set<LimbID>& /*limbAssociatedWithAction*/) { emit(std::make_unique<ExecuteKick>()); },
            [this](const std::set<LimbID>& /*limbAssociatedWithAction*/) { emit(std::make_unique<FinishKick>()); },
            [this](const std::set<ServoID>& /*servoAssociatedWithCompletion*/) {
                emit(std::make_unique<FinishKick>());
            }}));

        on<Trigger<KickScriptCommand>>().then([this](const KickScriptCommand& cmd) {
            kick_command       = std::make_shared<KickScriptCommand>(cmd);
            time_since_message = NUClear::clock::now();
            update_priority(cfg.kick_priority);
        });

        on<Trigger<ExecuteKick>>().then([this] {
            // Don't kick if there is no command
            // This may happen if we get priority initially with 0 priority and no KickScriptCommand
            if (kick_command == nullptr) {
                update_priority(0);
                return;
            }

            // Don't kick if it's been a while since we got the KickScriptCommand
            if (std::chrono::duration_cast<std::chrono::milliseconds>(NUClear::clock::now() - time_since_message)
                    .count()
                > cfg.message_timeout) {
                update_priority(0);
                return;
            }

            LimbID leg = kick_command->leg;

            // Execute the penalty kick if the type is PENALTY
            if (kick_command->type == KickCommandType::PENALTY) {
                emit(std::make_unique<ExecuteScriptByName>(subsumption_id,
                                                           std::vector<std::string>({"KickPenalty.yaml"})));
            }
            else {
                if (leg == LimbID::RIGHT_LEG) {
                    emit(std::make_unique<ExecuteScriptByName>(
                        subsumption_id,
                        std::vector<std::string>({"Stand.yaml", "KickRight.yaml", "Stand.yaml"})));
                }
                else {  // LEFT_LEG
                    emit(std::make_unique<ExecuteScriptByName>(
                        subsumption_id,
                        std::vector<std::string>({"Stand.yaml", "KickLeft.yaml", "Stand.yaml"})));
                }
            }
        });

        on<Trigger<FinishKick>>().then([this] {
            emit(std::make_unique<KickFinished>());
            update_priority(0);
        });
    }

    void KickScript::update_priority(const double& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumption_id, {priority}}));
    }

}  // namespace module::behaviour::skills
