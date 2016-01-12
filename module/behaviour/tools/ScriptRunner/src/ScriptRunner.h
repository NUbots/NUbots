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

#ifndef MODULES_BEHAVIOURS_UTILITY_SCRIPTRUNNER_H
#define MODULES_BEHAVIOURS_UTILITY_SCRIPTRUNNER_H

#include <nuclear>
#include <queue>

#include "message/motion/Script.h"

namespace module {
    namespace behaviour {
        namespace tools {

            /**
             * Executes a series of scripts provided by the command line in order.
             *
             * @author Trent Houliston
             */
            class ScriptRunner : public NUClear::Reactor {
            private:
                /// The scripts to be executed
                std::queue<std::string> scripts;

                /// Our ID for subsumption
                const size_t id;

                /// Execute the next script in the list
                void executeNextScript();
            public:
                explicit ScriptRunner(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // tools
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOURS_UTILITY_SCRIPTRUNNER_H

