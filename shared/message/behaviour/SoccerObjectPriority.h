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


#ifndef MESSAGE_BEHAVIOUR_SOCCEROBJECTPRIORITY_H
#define MESSAGE_BEHAVIOUR_SOCCEROBJECTPRIORITY_H

namespace message {
namespace behaviour {

	enum class SearchType {
        LOST,
        FIND_ADDITIONAL_OBJECTS,
        GOAL_SEARCH,
        GOAL_LEFT,
        GOAL_RIGHT,
        GROUND_LEFT,
        GROUND_RIGHT,
        OTHER
    };

    inline SearchType searchTypeFromString(std::string s){

        if(s.compare("LOST") == 0) {
            return SearchType::LOST;
        } else if(s.compare("FIND_ADDITIONAL_OBJECTS") == 0){
            return SearchType::FIND_ADDITIONAL_OBJECTS;
        } else if(s.compare("GOAL_SEARCH") == 0){
            return SearchType::GOAL_SEARCH;
        } else if(s.compare("GOAL_RIGHT") == 0){
            return SearchType::GOAL_RIGHT;
        } else if (s.compare("GOAL_LEFT") == 0) {
            return SearchType::GOAL_LEFT;
        } else if (s.compare("GROUND_RIGHT") == 0) {
            return SearchType::GROUND_RIGHT;
        } else if (s.compare("GROUND_LEFT") == 0) {
            return SearchType::GROUND_LEFT;
        } else if (s.compare("OTHER") == 0) {
            return SearchType::OTHER;
        } else {
            throw std::domain_error("HeadBehaviourSoccer - searchTypeFromString: NO SEARCH TYPE FOUND!");
        }

    }

	struct SoccerObjectPriority {
		int ball = 0;
		int goal = 0;
		int line = 0;

		SearchType searchType = SearchType::LOST;
	};

}
}

#endif
