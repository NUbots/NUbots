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

    enum class FieldObject { // TODO: definitely would be already defined elsewhere
        SELF,
        BALL,
        GOAL
    };

    class SoccerStrategy : public NUClear::Reactor {
    private:
        void standStill();
        void searchWalk();
        void walkTo(const FieldObject& object);
        void walkTo(arma::vec position);
        void lookAt(const FieldObject& object);
        void find(const std::vector<FieldObject>& objects);
        void spinWalk();
        bool pickedUp();
        bool penalised();
        bool recentlyVisible(const FieldObject& object);
        bool visible(const FieldObject& object);
        bool isGoalie();
        bool inZone(const FieldObject& object);
        bool isClose(const FieldObject& object);
        bool timePassed();
    public:
        static constexpr const char* CONFIGURATION_PATH = "SoccerStrategy.yaml";

        explicit SoccerStrategy(std::unique_ptr<NUClear::Environment> environment);
    };

}  // strategy
}  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H

