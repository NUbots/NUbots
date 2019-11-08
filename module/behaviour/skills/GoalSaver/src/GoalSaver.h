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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_PLANNING_GOALSAVER_H
#define MODULES_BEHAVIOUR_PLANNING_GOALSAVER_H

#include <nuclear>

#include "message/motion/DiveCommand.h"

namespace module {
namespace behaviour {
    namespace skills {

        class GoalSaver : public NUClear::Reactor {
        public:
            /// @brief Called by the powerplant to build and setup the GoalSaver reactor.
            explicit GoalSaver(std::unique_ptr<NUClear::Environment> environment);

        private:
            const size_t id;
            float DIVE_PRIORITY;
            float EXECUTION_PRIORITY;

            /**
             * TODO DiveCommand seems incompleted?
             * @Mingze
             * DiveCommand returns vec2 direction to dive
             */
            message::motion::DiveCommand diveCommand;

            void updatePriority(const float& priority);
            int getDirectionalQuadrant(float x, float y);
        };
    }  // namespace skills
}  // namespace behaviour
}  // namespace module


#endif
