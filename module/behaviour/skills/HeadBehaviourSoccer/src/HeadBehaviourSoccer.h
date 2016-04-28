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
#include "message/localisation/FieldObject.h"
#include "message/vision/VisionObjects.h"
#include "message/motion/HeadCommand.h"
#include "message/input/Sensors.h"
#include "message/input/CameraParameters.h"
#include "Searcher.h"
#include "message/behaviour/SoccerObjectPriority.h"

namespace module {
    namespace behaviour{
        namespace skills {

            /**
             * Executes a HeadBehaviourSoccer action.
             *
             * @author Jake Fountain
             */
            class HeadBehaviourSoccer : public NUClear::Reactor {
            public:


            private:

                /*! @brief Updates the search plan when something has changed
                */
                void updateHeadPlan(const std::vector<message::vision::VisionObject>& fixationObjects, const bool& search, const message::input::Sensors& sensors, const utility::math::matrix::Rotation3D& headToIMUSpace);

                /*! @brief Converts from camera space direction to IMU space direction
                */
                arma::vec2 getIMUSpaceDirection(const arma::vec2& screenAngles, utility::math::matrix::Rotation3D headToIMUSpace);

                /*! @brief Gets points which allow for simultaneous search and viewing of key objects
                */
                std::vector<arma::vec2> getSearchPoints(std::vector<message::vision::VisionObject> fixationObjects, message::behaviour::SearchType sType, const message::input::Sensors& sensors);

                /*! @brief Combines a collection of vision objects. The screen resulting screen angular region is the bounding box of the objects
                */
                message::vision::VisionObject combineVisionObjects(const std::vector<message::vision::VisionObject>& obs);

                /*! @brief Gets a bounding box in screen angular space of a set of vision objects
                */
                utility::math::geometry::Quad getScreenAngularBoundingBox(const std::vector<message::vision::VisionObject>& obs);

                bool orientationHasChanged(const message::input::Sensors& sensors);


                //CONFIG - loaded elsewhere
                float max_yaw;
                float min_yaw;
                float max_pitch;
                float min_pitch;


                float replan_angle_threshold;
                utility::math::matrix::Rotation3D lastPlanOrientation;

                message::input::CameraParameters cam;

                //CONFIG from HeadBehaviourSoccer.yaml
                float pitch_plan_threshold;
                double fractional_view_padding;
                float search_timeout_ms;
                float fractional_angular_update_threshold;

                bool oscillate_search;

                bool locBallReceived = false;
                message::localisation::Ball lastLocBall;

                std::map<message::behaviour::SearchType, std::vector<arma::vec2>> searches;

                //State variables
                Searcher<arma::vec2> headSearcher;

                int ballPriority = 0;
                int goalPriority = 0;
                message::behaviour::SearchType searchType = message::behaviour::SearchType::LOST;

                NUClear::clock::time_point lastPlanUpdate;
                NUClear::clock::time_point timeLastObjectSeen;

                arma::vec2 lastCentroid;

                bool lostAndSearching = false;
                bool lostLastTime = false;

                bool isGettingUp = false;

                int lastBallPriority = 0;
                int lastGoalPriority = 0;

            public:
                explicit HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // skills
    } //behaviour
}  // modules

#endif  // MODULES_BEHAVIOURS_REFLEX_HEADBEHAVIOURSOCCER_H

