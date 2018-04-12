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
#include <nuclear>

#include "KickScript.h"

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/behaviour/ServoCommand.h"
#include "message/motion/WalkCommand.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"

namespace module {
namespace behaviour {
    namespace skills {

        struct ExecuteKick {};
        struct FinishKick {};

        using extension::Configuration;
        using extension::ExecuteScriptByName;

        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        using message::motion::KickFinished;
        using message::motion::KickScriptCommand;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;

        KickScript::KickScript(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , id(size_t(this) * size_t(this) - size_t(this))
            , KICK_PRIORITY(0.0f)
            , EXECUTION_PRIORITY(0.0f)
            , kickCommand() {

            // do a little configurating
            on<Configuration>("KickScript.yaml").then([this](const Configuration& config) {
                KICK_PRIORITY      = config["KICK_PRIORITY"].as<float>();
                EXECUTION_PRIORITY = config["EXECUTION_PRIORITY"].as<float>();
            });

            on<Trigger<KickScriptCommand>>().then([this](const KickScriptCommand& kickCommand) {
                auto direction = kickCommand.direction;
                LimbID leg     = kickCommand.leg;

                int quadrant = getDirectionalQuadrant(direction[0], direction[1]);

                // check if the command was valid
                bool valid = true;
                if (leg == LimbID::RIGHT_LEG) {
                    if (quadrant == 2 || quadrant == 3) {
                        NUClear::log<NUClear::WARN>("Right leg cannot kick towards: ", direction);
                        valid = false;
                    }
                }
                else if (leg == LimbID::LEFT_LEG) {
                    if (quadrant == 2 || quadrant == 1) {
                        NUClear::log<NUClear::WARN>("Left leg cannot kick towards: ", direction);
                        valid = false;
                    }
                }
                else {
                    NUClear::log<NUClear::WARN>("Cannot kick with limb: ", leg);
                    updatePriority(0);
                    valid = false;
                }

                if (valid) {
                    this->kickCommand = kickCommand;
                    updatePriority(KICK_PRIORITY);
                }
            });

            on<Trigger<ExecuteKick>>().then([this] {
                // auto direction = kickCommand.direction;
                LimbID leg = kickCommand.leg;

                if (leg == LimbID::RIGHT_LEG) {
                    emit(std::make_unique<ExecuteScriptByName>(
                        id, std::vector<std::string>({"Stand.yaml", "KickRight.yaml", "Stand.yaml"})));
                }
                else {  // if (leg == LimbID::LEFT_LEG) {
                    emit(std::make_unique<ExecuteScriptByName>(
                        id, std::vector<std::string>({"Stand.yaml", "KickLeft.yaml", "Stand.yaml"})));
                }

                // if (kickCommand.kickCommandType == KickType::SCRIPTED) {
                // } else {
                // int quadrant = getDirectionalQuadrant(direction[0], direction[1]);
                // // assume valid at this point as this is checked on the walkcommand trigger
                // if (leg == LimbID::RIGHT_LEG) {
                //     if (quadrant == 0) {
                //         // front
                //         emit(std::make_unique<ExecuteScriptByName>(id,
                //         std::vector<std::string>({"RightFootKickForward3.yaml", "Stand.yaml"})));
                //     } else if (quadrant == 1) {
                //         // side
                //         emit(std::make_unique<ExecuteScriptByName>(id,
                //         std::vector<std::string>({"RightFootKickLeft.yaml", "Stand.yaml"})));
                //     }
                // } else if (leg == LimbID::LEFT_LEG) {
                //     if (quadrant == 0) {
                //         // front
                //         emit(std::make_unique<ExecuteScriptByName>(id,
                //         std::vector<std::string>({"LeftFootKickForward3.yaml", "Stand.yaml"})));
                //     } else if (quadrant == 3) {
                //         // side
                //         emit(std::make_unique<ExecuteScriptByName>(id,
                //         std::vector<std::string>({"LeftFootKickRight.yaml", "Stand.yaml"})));
                //     }
                // }
                // }

                updatePriority(EXECUTION_PRIORITY);

            });

            on<Trigger<FinishKick>>().then([this] {
                emit(std::move(std::make_unique<KickFinished>()));
                updatePriority(0);
            });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{id,
                               "Kick Script",
                               {std::pair<float, std::set<LimbID>>(
                                   0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
                               [this](const std::set<LimbID>&) { emit(std::make_unique<ExecuteKick>()); },
                               [this](const std::set<LimbID>&) { emit(std::make_unique<FinishKick>()); },
                               [this](const std::set<ServoID>&) { emit(std::make_unique<FinishKick>()); }}));
        }

        void KickScript::updatePriority(const float& priority) {
            emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {priority}}));
        }

        int KickScript::getDirectionalQuadrant(float x, float y) {

            // These represent 4 directions of looking, see https://www.desmos.com/calculator/mm8cnsnpdt for a graph of
            // the 4 quadrants
            // Note that x is forward in relation to the robot so the forward quadrant is x >= |y|
            return x >= std::abs(y) ? 0                                          // forward
                                    : y >= std::abs(x) ? 1                       // left
                                                       : x <= -std::abs(y) ? 2   // backward
                                                                           : 3;  // right
        }
    }  // namespace skills
}  // namespace behaviour
}  // namespace module
