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

#ifndef MODULES_BEHAVIOUR_REFLEX_HEADBEHAVIOURSOCCER_H
#define MODULES_BEHAVIOUR_REFLEX_HEADBEHAVIOURSOCCER_H

#include <nuclear>
#include <armadillo>
#include <set>
#include "messages/vision/VisionObjects.h"
#include "messages/motion/HeadCommand.h"


namespace modules {
    namespace behaviour{
        namespace skills {

            /**
             * Executes a HeadBehaviourSoccer action.
             *
             * @author Josiah Walker
             */
            class HeadBehaviourSoccer : public NUClear::Reactor {
            private:
                void updateHeadPlan(const std::vector<std::unique_ptr<messages::vision::VisionObject>>& fixationObjects, const bool& search);
                std::unique_ptr<messages::motion::HeadCommand> getHeadCommand();

                // int ballsSeenLastUpdate;
                // int goalPostsSeenLastUpdate;
                // time_t lastUpdateTime;
            public:
                explicit HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment);
                static constexpr const char* CONFIGURATION_PATH = "HeadBehaviourSoccer.yaml";
            };

        }  // motion
    } //behaviour
}  // modules

#endif  // MODULES_BEHAVIOURS_REFLEX_HEADBEHAVIOURSOCCER_H

