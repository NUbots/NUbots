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
#include "messages/input/Sensors.h"
#include "messages/input/CameraParameters.h"


namespace modules {
    namespace behaviour{
        namespace skills {

            /**
             * Executes a HeadBehaviourSoccer action.
             *
             * @author Josiah Walker
             */
            class HeadBehaviourSoccer : public NUClear::Reactor {
            public:
                enum SearchType {
                    LOW_FIRST = 0,
                    HIGH_FIRST = 1,
                    CROSS = 2,
                    OTHER = 3
                };
                SearchType searchTypeFromString(std::string s){

                    if(s.compare("LOW_FIRST") == 0) {
                        return SearchType::LOW_FIRST;
                    }
                    else if(s.compare("HIGH_FIRST") == 0) {
                        return SearchType::HIGH_FIRST;
                    }
                    else if(s.compare("CROSS") == 0) {
                        return SearchType::CROSS;
                    }
                    else {
                        return SearchType::OTHER;
                    }


                }

            private:
                void updateHeadPlan(const std::vector<messages::vision::VisionObject>& fixationObjects, const bool& search, const messages::input::Sensors& sensors);
                std::unique_ptr<messages::motion::HeadCommand> getHeadCommand();

                std::vector<arma::vec2> getSearchPoints(std::vector<arma::vec2> fixationPoints, std::vector<arma::vec2> fixationSizes, SearchType sType);

                float currentWorldPitch;
                float currentWorldYaw;

                float max_yaw;
                float min_yaw;
                float max_pitch;
                float min_pitch;

                float p_gain_tracking;

                double view_padding_radians;

                messages::input::CameraParameters cam;

                std::map<SearchType, std::vector<arma::vec2>> lost_searches;

                int debug_look_index;

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

