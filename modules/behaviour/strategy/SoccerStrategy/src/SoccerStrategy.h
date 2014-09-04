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
#include <armadillo>

#include "messages/behaviour/FieldTarget.h"

namespace modules {
namespace behaviour {
namespace strategy {

    class SoccerStrategy : public NUClear::Reactor {
    private:
        messages::behaviour::FieldTarget walkTarget;
        std::vector<messages::behaviour::FieldTarget> lookTarget;

        double BALL_CLOSE_DISTANCE;
        NUClear::clock::duration BALL_LAST_SEEN_MAX_TIME;
        NUClear::clock::duration GOAL_LAST_SEEN_MAX_TIME;

        // TODO: remove horrible
        bool isGettingUp = false;
        bool isDiving = false;
        bool selfPenalised = false;

        // TODO: initalize
        struct Zone {
            NUClear::clock::duration ballActiveTimeout;
            NUClear::clock::duration zoneReturnTimeout;
            arma::mat22 zone;
            arma::vec2 startPositionOffensive;
            arma::vec2 startPositionDefensive;
            arma::vec2 defaultPosition;
            bool goalie;
        } zone;

        time_t ballLastSeen;
        time_t goalLastSeen;
        std::string leaf = "";
        void resetRobotHypotheses();

        void standStill();
        void searchWalk();
        void walkTo(const messages::behaviour::FieldTarget& object);
        void walkTo(arma::vec position);
        void find(const std::vector<messages::behaviour::FieldTarget>& objects);
        void spinWalk();
        bool pickedUp();
        bool penalised();
        bool isGoalie();
        bool inZone(const messages::behaviour::FieldTarget& object);
        bool ballDistance();
    public:
        static constexpr const char* CONFIGURATION_PATH = "SoccerStrategy.yaml";

        explicit SoccerStrategy(std::unique_ptr<NUClear::Environment> environment);
    };

}  // strategy
}  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H

