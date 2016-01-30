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

#ifndef MODULES_BEHAVIOUR_REFLEX_GETUP_H
#define MODULES_BEHAVIOUR_REFLEX_GETUP_H

#include <nuclear>

namespace module {
    namespace behaviour {
        namespace skills {

            /**
             * Executes a getup script if the robot falls over.
             *
             * @author Josiah Walker
             * @author Trent Houliston
             */
            class Getup : public NUClear::Reactor {
            private:
                const size_t id;

                bool gettingUp;
                ReactionHandle fallenCheck;

                /// config settings
                float FALLEN_ANGLE;
                float GETUP_PRIORITY;
                float EXECUTION_PRIORITY;

                void updatePriority(const float& priority);

            public:
                explicit Getup(std::unique_ptr<NUClear::Environment> environment);
            };
        }  // skills
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOURS_UTILITY_SCRIPTRUNNER_H

