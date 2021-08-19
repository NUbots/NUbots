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

    KickScript::KickScript(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // do a little configurating
        on<Configuration>("KickScript.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            KICK_PRIORITY      = config["KICK_PRIORITY"].as<float>();
            EXECUTION_PRIORITY = config["EXECUTION_PRIORITY"].as<float>();
        });

        on<Trigger<KickScriptCommand>>().then([this](const KickScriptCommand& cmd) {
            kickCommand = cmd;
            updatePriority(KICK_PRIORITY);
        });

        on<Trigger<ExecuteKick>>().then([this] {
            // auto direction = kickCommand.direction;
            LimbID leg = kickCommand.leg;

            // Execute the penalty kick if the type is PENALTY
            if (kickCommand.type == KickCommandType::PENALTY) {
                emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"KickPenalty.yaml"})));
            }
            else {
                if (leg == LimbID::RIGHT_LEG) {
                    emit(std::make_unique<ExecuteScriptByName>(
                        id,
                        std::vector<std::string>({"Stand.yaml", "KickRight.yaml", "Stand.yaml"})));
                }
                else {  // if (leg == LimbID::LEFT_LEG) {
                    emit(std::make_unique<ExecuteScriptByName>(
                        id,
                        std::vector<std::string>({"Stand.yaml", "KickLeft.yaml", "Stand.yaml"})));
                }
            }

            updatePriority(EXECUTION_PRIORITY);
        });

        on<Trigger<FinishKick>>().then([this] {
            emit(std::make_unique<KickFinished>());
            updatePriority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            id,
            "Kick Script",
            {std::pair<float, std::set<LimbID>>(
                0,
                {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
            [this](const std::set<LimbID>& /*limbAssociatedWithAction*/) { emit(std::make_unique<ExecuteKick>()); },
            [this](const std::set<LimbID>& /*limbAssociatedWithAction*/) { emit(std::make_unique<FinishKick>()); },
            [this](const std::set<ServoID>& /*servoAssociatedWithCompletion*/) {
                emit(std::make_unique<FinishKick>());
            }}));
    }

    void KickScript::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{id, {priority}}));
    }

}  // namespace module::behaviour::skills
