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

#include "Searcher.h"

#include "message/localisation/proto/FieldObject.h"
#include "message/vision/proto/VisionObjects.h"
#include "message/motion/proto/HeadCommand.h"
#include "message/input/proto/Sensors.h"
#include "message/motion/proto/KinematicsModels.h"
#include "message/input/proto/CameraParameters.h"
#include "message/behaviour/proto/SoccerObjectPriority.h"

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
                enum SearchState{
                    FIXATION = 0,
                    WAIT = 1,
                    SEARCH = 2
                };
                SearchState state = SearchState::SEARCH;

                std::vector<message::vision::proto::Ball> getFixationObjects(std::shared_ptr<const std::vector<message::vision::proto::Ball>> vballs, bool& search);
                std::vector<message::vision::proto::Goal> getFixationObjects(std::shared_ptr<const std::vector<message::vision::proto::Goal>> vgoals, bool& search);


                /*! @brief Updates the search plan when something has changed
                */
                void updateHeadPlan(const message::motion::proto::KinematicsModel& kinematicsModel, const std::vector<message::vision::proto::Ball>& fixationObjects, const bool& search, const message::input::proto::Sensors& sensors, const utility::math::matrix::Rotation3D& headToIMUSpace);
                void updateHeadPlan(const message::motion::proto::KinematicsModel& kinematicsModel, const std::vector<message::vision::proto::Goal>& fixationObjects, const bool& search, const message::input::proto::Sensors& sensors, const utility::math::matrix::Rotation3D& headToIMUSpace);

                /*! @brief Converts from camera space direction to IMU space direction
                */
                arma::vec2 getIMUSpaceDirection(const message::motion::proto::KinematicsModel& kinematicsModel, const arma::vec2& screenAngles, utility::math::matrix::Rotation3D headToIMUSpace);

                /*! @brief Gets points which allow for simultaneous search and viewing of key objects
                */
                std::vector<arma::vec2> getSearchPoints(const message::motion::proto::KinematicsModel& kinematicsModel, std::vector<message::vision::proto::Ball> fixationObjects, message::behaviour::proto::SearchType sType, const message::input::proto::Sensors& sensors);
                std::vector<arma::vec2> getSearchPoints(const message::motion::proto::KinematicsModel& kinematicsModel, std::vector<message::vision::proto::Goal> fixationObjects, message::behaviour::proto::SearchType sType, const message::input::proto::Sensors& sensors);

                /*! @brief Combines a collection of vision objects. The screen resulting screen angular region is the bounding box of the objects
                */
                message::vision::proto::Ball combineVisionObjects(const std::vector<message::vision::proto::Ball>& obs);
                message::vision::proto::Goal combineVisionObjects(const std::vector<message::vision::proto::Goal>& obs);

                /*! @brief Gets a bounding box in screen angular space of a set of vision objects
                */
                utility::math::geometry::Quad getScreenAngularBoundingBox(const std::vector<message::vision::proto::Ball>& obs);
                utility::math::geometry::Quad getScreenAngularBoundingBox(const std::vector<message::vision::proto::Goal>& obs);

                bool orientationHasChanged(const message::input::proto::Sensors& sensors);


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
                float pitch_plan_value = 20;
                double fractional_view_padding;
                float search_timeout_ms;
                float fractional_angular_update_threshold;

                bool oscillate_search;

                bool locBallReceived = false;
                message::localisation::proto::Ball lastLocBall;

                std::map<message::behaviour::proto::SearchType, std::vector<arma::vec2>> searches;

                //State variables
                Searcher<arma::vec2> headSearcher;

                int ballPriority = 0;
                int goalPriority = 0;
                message::behaviour::proto::SearchType searchType = message::behaviour::proto::SearchType::LOST;

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

