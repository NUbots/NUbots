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

#ifndef MODULES_BEHAVIOUR_PLANNING_LOOKPLANNER_H
#define MODULES_BEHAVIOUR_PLANNING_LOOKPLANNER_H

#include <nuclear>
#include <armadillo>
#include "messages/behaviour/LookStrategy.h"

namespace modules {
namespace behaviour {
namespace planning {

    class LookPlanner : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and setup the LookPlanner reactor.
        explicit LookPlanner(std::unique_ptr<NUClear::Environment> environment);
        static constexpr const char* CONFIGURATION_PATH = "LookPlanner.yaml";

    private:
        //configurable timeouts
        double VISUAL_TRACKING_TIMEOUT;
        double LOCALISATION_TRACKING_TIMEOUT;

        //storage for intermediate positions
        std::vector<std::pair<arma::vec2,arma::vec2>> ballObjects;
        std::vector<arma::vec2> ballPanPoints;
        std::vector<std::pair<arma::vec2,arma::vec2>> goalObjects;
        std::vector<arma::vec2> goalPanPoints;

        std::vector<arma::vec2> lostPanPoints;

        //timer for when we last saw the object
        std::chrono::system_clock::time_point timeBallSeen;
        std::chrono::system_clock::time_point timeGoalSeen;

        void updateLookPlan(const messages::behaviour::LookStrategy& strat);
    };

}
}
}


#endif
