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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_STRATEGY_KICKATGOAL_H
#define MODULES_BEHAVIOUR_STRATEGY_KICKATGOAL_H

#include <nuclear>

#include "messages/behaviour/proto/Behaviour.pb.h"

namespace modules {
namespace behaviour {
namespace strategy {

    class KickAtGoal : public NUClear::Reactor {
    private:
        NUClear::clock::duration ballActiveTimeout;
        time_t ballLastSeen;
        time_t goalLastSeen;

        void doBehaviour();
        void walkToBall();
        void spinToWin();

        messages::behaviour::proto::Behaviour::State currentState = messages::behaviour::proto::Behaviour::INIT;
    public:
        static constexpr const char* CONFIGURATION_PATH = "KickAtGoal.yaml";

        /// @brief Called by the powerplant to build and setup the KickAtGoal reactor.
        explicit KickAtGoal(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}


#endif
