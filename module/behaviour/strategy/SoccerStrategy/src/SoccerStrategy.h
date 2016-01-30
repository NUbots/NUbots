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

#include "message/behaviour/KickPlan.h"
#include "message/behaviour/FieldTarget.h"
#include "message/behaviour/proto/Behaviour.pb.h"
#include "message/localisation/FieldObject.h"
#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "message/input/gameevents/GameEvents.h"

namespace module {
namespace behaviour {
namespace strategy {

    class SoccerStrategy : public NUClear::Reactor {
    private:

        struct Config {
            NUClear::clock::duration ball_last_seen_max_time;
            NUClear::clock::duration goal_last_seen_max_time;

            float ball_search_walk_start_speed;
            float ball_search_walk_stop_speed;
            float ball_search_walk_slow_time;

            arma::vec2 start_position_offensive;
            arma::vec2 start_position_defensive;
            bool is_goalie;

            float goalie_command_timeout;
            float goalie_rotation_speed_factor;
            float goalie_max_rotation_speed;
            float goalie_translation_speed_factor;
            float goalie_max_translation_speed;
            float goalie_side_walk_angle_threshold;
            NUClear::clock::duration localisation_interval;
            NUClear::clock::duration localisation_duration;
            bool alwaysPowerKick;
            bool forcePlaying = false;
            bool forcePenaltyShootout = false;
        } cfg_;

        message::behaviour::FieldTarget walkTarget;

        std::vector<message::behaviour::FieldTarget> lookTarget;

        // TODO: remove horrible
        bool isGettingUp = false;
        bool isDiving = false;
        bool selfPenalised = false;
        bool isSideChecking = false;
        message::behaviour::KickType kickType;
        message::behaviour::proto::Behaviour::State currentState = message::behaviour::proto::Behaviour::INIT;

        NUClear::clock::time_point lastLocalised = NUClear::clock::now();

        NUClear::clock::time_point ballLastMeasured = NUClear::clock::now() - std::chrono::seconds(600); // TODO: unhack
        NUClear::clock::time_point ballSearchStartTime;
        NUClear::clock::time_point selfLastMeasured;
        void initialLocalisationReset(const message::support::FieldDescription& fieldDescription);
        void penaltyLocalisationReset();
        void unpenalisedLocalisationReset(const message::support::FieldDescription& fieldDescription);

        void standStill();
        void searchWalk();
        void walkTo(const message::support::FieldDescription& fieldDescription, const message::behaviour::FieldTarget& object);
        void walkTo(const message::support::FieldDescription& fieldDescription, arma::vec position);
        void find(const std::vector<message::behaviour::FieldTarget>& objects);
        void spinWalk();
        bool pickedUp(const message::input::Sensors& sensors);
        bool penalised();
        bool ballDistance(const message::localisation::Ball& ball);
        void goalieWalk(const std::vector<message::localisation::Self>& selfs, const std::vector<message::localisation::Ball>& balls);
        arma::vec2 getKickPlan(const std::vector<message::localisation::Self>& selfs, const message::support::FieldDescription& fieldDescription);
        void play(const std::vector<message::localisation::Self>& selfs, const std::vector<message::localisation::Ball>& balls, const message::support::FieldDescription& fieldDescription, const message::input::gameevents::Mode& mode);

    public:
        explicit SoccerStrategy(std::unique_ptr<NUClear::Environment> environment);
    };

}  // strategy
}  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H

