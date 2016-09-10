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

#ifndef MODULES_BEHAVIOUR_REFLEX_FALLINGRELAX_H
#define MODULES_BEHAVIOUR_REFLEX_FALLINGRELAX_H

#include <nuclear>

namespace module {
    namespace behaviour {
        namespace skills {

            /**
             * Executes a falling script if the robot falls over.
             *
             * @author Trent Houliston
             */
            class FallingRelax : public NUClear::Reactor {
            private:
                const size_t id;

                bool falling;

                /// config settings
                float FALLING_ANGLE;
                float FALLING_ACCELERATION;
                std::vector<float> RECOVERY_ACCELERATION;
                float PRIORITY;

                void updatePriority(const float& priority);

            public:
                explicit FallingRelax(std::unique_ptr<NUClear::Environment> environment);
            };
        }  // skills
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_REFLEX_FALLINGRELAX_H

