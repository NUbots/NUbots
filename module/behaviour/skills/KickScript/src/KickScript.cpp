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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "KickScript.h"
#include <nuclear>

#include "message/input/ServoID.h"
#include "message/motion/Script.h"
#include "message/behaviour/Action.h"
#include "message/behaviour/ServoCommand.h"
#include "message/support/Configuration.h"
#include "message/motion/WalkCommand.h"

namespace module {
namespace behaviour {
namespace skills {

    struct ExecuteKick{};
    struct FinishKick{};

    using message::support::Configuration;
    using message::input::ServoID;
    using message::motion::ExecuteScriptByName;
    using message::behaviour::RegisterAction;
    using message::behaviour::ActionPriorites;
    using message::motion::WalkStartCommand;
    using message::input::LimbID;
    using message::motion::KickScriptCommand;
    using message::motion::KickFinished;

    KickScript::KickScript(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        // do a little configurating
        on<Configuration>("KickScript.yaml").then([this] (const Configuration& config) {
            KICK_PRIORITY = config["KICK_PRIORITY"].as<float>();
            EXECUTION_PRIORITY = config["EXECUTION_PRIORITY"].as<float>();
        });

        on<Trigger<KickScriptCommand>>().then([this] (const KickScriptCommand& kickCommand) {
            auto direction = kickCommand.direction;
            auto leg = kickCommand.leg;

            int quadrant = getDirectionalQuadrant(direction[0], direction[1]);

            // check if the command was valid
            bool valid = true;
            if (leg == LimbID::RIGHT_LEG) {
                if (quadrant == 2 || quadrant == 3) {
                    NUClear::log<NUClear::WARN>("Right leg cannot kick towards: ", direction);
                    valid = false;
                }
            } else if (leg == LimbID::LEFT_LEG) {
                if (quadrant == 2 || quadrant == 1) {
                    NUClear::log<NUClear::WARN>("Left leg cannot kick towards: ", direction);
                    valid = false;
                }
            } else {
                NUClear::log<NUClear::WARN>("Cannot kick with limb: ", uint(leg));
                updatePriority(0);
                valid = false;
            }

            if (valid) {
                this->kickCommand = kickCommand;
                updatePriority(KICK_PRIORITY);
            }
        });

        on<Trigger<ExecuteKick>>().then([this] {
            auto direction = kickCommand.direction;
            auto leg = kickCommand.leg;

            if (leg == LimbID::RIGHT_LEG) {
                emit(std::make_unique<ExecuteScriptByName>(id,  std::vector<std::string>({"RightFootPowerKick.yaml"})));
            } else { //if (leg == LimbID::LEFT_LEG) {
                emit(std::make_unique<ExecuteScriptByName>(id,  std::vector<std::string>({"LeftFootPowerKick.yaml"})));
            }

            // if (kickCommand.kickCommandType == KickType::SCRIPTED) {
            // } else {
                // int quadrant = getDirectionalQuadrant(direction[0], direction[1]);
                // // assume valid at this point as this is checked on the walkcommand trigger
                // if (leg == LimbID::RIGHT_LEG) {
                //     if (quadrant == 0) {
                //         // front
                //         emit(std::make_unique<ExecuteScriptByName>(id,  std::vector<std::string>({"RightFootKickForward3.yaml", "Stand.yaml"})));
                //     } else if (quadrant == 1) {
                //         // side
                //         emit(std::make_unique<ExecuteScriptByName>(id,  std::vector<std::string>({"RightFootKickLeft.yaml", "Stand.yaml"})));
                //     }
                // } else if (leg == LimbID::LEFT_LEG) {
                //     if (quadrant == 0) {
                //         // front
                //         emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"LeftFootKickForward3.yaml", "Stand.yaml"})));
                //     } else if (quadrant == 3) {
                //         // side
                //         emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"LeftFootKickRight.yaml", "Stand.yaml"})));
                //     }
                // }
            // }

            updatePriority(EXECUTION_PRIORITY);

        });

        on<Trigger<FinishKick>>().then([this] {
            emit(std::move(std::make_unique<KickFinished>()));
            updatePriority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "Kick Script",
            { std::pair<float, std::set<LimbID>>(0, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM }) },
            [this] (const std::set<LimbID>&) {
                emit(std::make_unique<ExecuteKick>());
            },
            [this] (const std::set<LimbID>&) {
                emit(std::make_unique<FinishKick>());
            },
            [this] (const std::set<ServoID>&) {
                emit(std::make_unique<FinishKick>());
            }
        }));
    }

    void KickScript::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { priority }}));
    }

    int KickScript::getDirectionalQuadrant(float x, float y) {

            // These represent 4 directions of looking, see https://www.desmos.com/calculator/mm8cnsnpdt for a graph of the 4 quadrants
            // Note that x is forward in relation to the robot so the forward quadrant is x >= |y|
            return x >=  std::abs(y) ? 0  // forward
                 : y >=  std::abs(x) ? 1  // left
                 : x <= -std::abs(y) ? 2  // backward
                 :                     3; // right
    }
}
}
}

