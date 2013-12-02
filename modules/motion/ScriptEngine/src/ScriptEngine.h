/*
 * This file is part of ScriptEngine.
 *
 * ScriptEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_MOTION_SCRIPTENGINE_H
#define MODULES_MOTION_SCRIPTENGINE_H

#include <nuclear>
#include "messages/Script.h"

namespace modules {
    namespace motion {

        /**
         * Executes scripts as a series of waypoints.
         * Can either find a script by name, or take a script as a set of waypoints
         * 
         * @author Trent Houliston
         */
        class ScriptEngine : public NUClear::Reactor {
        private:
            std::map<std::string, messages::Script> scripts;
        public:
            explicit ScriptEngine(std::unique_ptr<NUClear::Environment> environment);
        };
    
    }  // motion
}  // modules

#endif  // MODULES_MOTION_SCRIPTENGINE_H

