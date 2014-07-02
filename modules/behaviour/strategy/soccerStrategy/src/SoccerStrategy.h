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

		typedef struct {
			bool selfInZone;
			bool ballInZone;
			bool goalInRange;
			bool kicker;
			bool pickedUp;
			bool putDown;
			bool penalised;
			bool unPenalised;
			bool kickOff;
			bool gameStateInitial;
			bool gameStateSet;
			bool gameStateReady;
			bool gameStateFinish;
			bool gameStatePlaying;
			bool ballSeen;
			bool ballLost;
			bool teamBallSeen;
			bool ballApproaching;
			bool ballApproachingGoal;
			bool kickPosition;

			arma::vec2 currentHeading;
			arma::vec2 currentPosition;
			arma::vec2 targetHeading;
			arma::vec2 targetPosition;
		} State;

		/**
		* High level behaviour for robot soccer.
		*
		* @author Alex Biddulph
		*/
		class SoccerStrategy : public NUClear::Reactor {
		private:
			NUClear::clock::time_point timeSinceLastSeen;

			std::vector<arma::vec2> MY_ZONE;
			float MAX_BALL_DISTANCE;
			float KICK_DISTANCE_THRESHOLD;
			float BALL_CERTAINTY_THRESHOLD;
			arma::vec2 START_POSITION;
			bool IS_GOALIE;

			State previousState, currentState;

		public:
			explicit SoccerStrategy(std::unique_ptr<NUClear::Environment> environment);

			static constexpr const char* CONFIGURATION_PATH = "SoccerStrategy.yaml";
			};

		}  // strategy
	}  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H

