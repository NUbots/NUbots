/*
 * This file is part of Behaviour Controller.
 *
 * Behaviour Controller is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Behaviour Controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Behaviour Controller.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_CONTROLLER_H
#define MODULES_BEHAVIOUR_CONTROLLER_H

#include <nuclear>

#include "messages/behaviour/Action.h"

namespace modules {
    namespace behaviour {

        struct RequestItem;

        struct Request {
            bool active;
            std::vector<std::unique_ptr<RequestItem>> items;
        };

        struct RequestItem {
            Request& parent;

            bool active;
            bool mainElement;

            float priority;
            std::set<messages::behaviour::LimbID> limbSet;
        };

        /**
         * Controls which of the behaviours are able to access motors.
         * 
         * @author Trent Houliston
         */
        class Controller : public NUClear::Reactor {
        private:
            std::set<RequestItem> actions[5];
            std::map<size_t, std::unique_ptr<Request>> requests;
            
            void selectAction();
        public:
            explicit Controller(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_CONTROLLER_H

