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

#include "messages/input/ServoID.h"
#include "messages/motion/Script.h"
#include "messages/behaviour/Action.h"
#include "messages/support/Configuration.h"

namespace modules {
namespace behaviour {
namespace reflexes {

    struct ExecuteKick{};
    struct FinishKick{};
    
    using messages::support::Configuration;
    using messages::input::ServoID;
    using messages::motion::ExecuteScriptByName;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;
    using messages::behaviour::LimbID;
    using messages::motion::KickCommand;

    KickScript::KickScript(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        // do a little configurating
        on<Trigger<Configuration<KickScript>>>([this] (const Configuration<KickScript>& config){
            KICK_PRIORITY = config["KICK_PRIORITY"];
            EXECUTION_PRIORITY = config["EXECUTION_PRIORITY"];
        });

        on<Trigger<KickCommand>>([this] (const KickCommand& kickCommand) {
            auto direction = kickCommand.direction;
            auto leg = kickCommand.leg;
            float x = direction[0];
            float y = direction[1];

            int quadrant = x >=  std::abs(y) ? 0
                         : y >=  std::abs(x) ? 1
                         : x <= -std::abs(y) ? 2
                         : 3;

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

        on<Trigger<ExecuteKick>>([this] (const ExecuteKick&) {
            auto direction = kickCommand.direction;
            auto leg = kickCommand.leg;
            float x = direction[0];
            float y = direction[1];

            int quadrant = x >=  std::abs(y) ? 0
                         : y >=  std::abs(x) ? 1
                         : x <= -std::abs(y) ? 2
                         : 3;
            // assume valid at this point

            if (leg == LimbID::RIGHT_LEG) {
                if (quadrant == 0) {
                    // front
                    emit(std::make_unique<ExecuteScriptByName>(id, "KickRight.json"));
                } else if (quadrant == 1) {
                    // side
                    emit(std::make_unique<ExecuteScriptByName>(id, "RightFootKickLeft.json"));
                }
            } else if (leg == LimbID::LEFT_LEG) {
                if (quadrant == 0) {
                    // front
                    emit(std::make_unique<ExecuteScriptByName>(id, "KickLeft.json"));
                } else if (quadrant == 3) {
                    // side
                    emit(std::make_unique<ExecuteScriptByName>(id, "LeftFootKickRight.json"));
                }
            }

            updatePriority(EXECUTION_PRIORITY);

        });

        on<Trigger<FinishKick>>([this] (const FinishKick&) {
            updatePriority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
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

}
}
}

