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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_SIDECHECKER_H
#define MODULES_LOCALISATION_SIDECHECKER_H

#include <nuclear>

#include "messages/vision/VisionObjects.h"


namespace modules {
namespace localisation {

    class SideChecker : public NUClear::Reactor {
    private:
    	struct Config {
    		float priority;
    		int number_of_samples;
    	} cfg_;

    	enum State {
    		SearchLeft,
    		SearchRight,
    		Calculate
    	};

    	//Collected data:
    	std::vector<std::pair<messages::vision::Goal,messages::vision::Goal>> leftGoals; //list of (left,right) goals
    	std::vector<std::pair<messages::vision::Goal,messages::vision::Goal>> rightGoals; //list of (left,right) goals

    	State currentState = State::SearchLeft;
    	void addGoals(messages::vision::Goal left, messages::vision::Goal right);

    public:
        /// @brief Called by the powerplant to build and setup the SideChecker reactor.
        explicit SideChecker(std::unique_ptr<NUClear::Environment> environment);
        const size_t subsumptionId;
        /// @brief the path to the configuration file for SideChecker
        static constexpr const char* CONFIGURATION_PATH = "SideChecker.yaml";
    };

}
}

#endif  // MODULES_LOCALISATION_SIDECHECKER_H
