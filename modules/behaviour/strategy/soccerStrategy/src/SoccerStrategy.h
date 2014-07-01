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

#ifndef MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H
#define MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H

#include <nuclear>

namespace modules {
    namespace behaviour {
        namespace strategy {

            /**
             * High level behaviour for robot soccer.
             *
             * @author Alex Biddulph
             */
            class SoccerStrategy : public NUClear::Reactor {
            private:
                NUClear::clock::time_point timeSinceLastSeen;

                std::vector<arma::vec> MY_ZONE;
		int MAX_BALL_DISTANCE;

            public:
                explicit SoccerStrategy(std::unique_ptr<NUClear::Environment> environment);

                static constexpr const char* CONFIGURATION_PATH = "SoccerStrategy.yaml";
            };

        }  // strategy
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H

