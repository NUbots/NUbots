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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_REFLEX_HEADBEHAVIOURSOCCER_HPP
#define MODULES_BEHAVIOUR_REFLEX_HEADBEHAVIOURSOCCER_HPP

#include <nuclear>
#include <set>

#include "Searcher.hpp"

#include "message/behaviour/SoccerObjectPriority.hpp"
#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/motion/HeadCommand.hpp"
#include "message/motion/KinematicsModel.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Goal.hpp"

#include "utility/math/geometry/Quad.hpp"

namespace module::behaviour::skills {

    /**
     * Executes a HeadBehaviourSoccer action.
     *
     * @author Jade Fountain
     */
    class HeadBehaviourSoccer : public NUClear::Reactor {
    public:
    private:
        enum SearchState { FIXATION = 0, WAIT = 1, SEARCH = 2 };
        SearchState state = SearchState::SEARCH;

        message::vision::Balls getFixationObjects(const std::shared_ptr<const message::vision::Balls>& vballs,
                                                  bool& search);
        message::vision::Goals getFixationObjects(const std::shared_ptr<const message::vision::Goals>& vgoals,
                                                  bool& search);


        /*! @brief Updates the search plan when something has changed
         */
        void updateHeadPlan(const message::motion::KinematicsModel& kinematicsModel,
                            const message::vision::Balls& fixationObjects,
                            const bool& search,
                            const message::input::Sensors& sensors,
                            const Eigen::Matrix3d& headToIMUSpace,
                            const message::input::Image::Lens& lens);
        void updateHeadPlan(const message::motion::KinematicsModel& kinematicsModel,
                            const message::vision::Goals& fixationObjects,
                            const bool& search,
                            const message::input::Sensors& sensors,
                            const Eigen::Matrix3d& headToIMUSpace,
                            const message::input::Image::Lens& lens);

        /*! @brief Converts from camera space direction to IMU space direction
         */
        static Eigen::Vector2d getIMUSpaceDirection(const Eigen::Vector2d& screenAngles,
                                                    const Eigen::Matrix3d& headToIMUSpace);

        /*! @brief Gets points which allow for simultaneous search and viewing of key objects
         */
        std::vector<Eigen::Vector2d> getSearchPoints(const message::motion::KinematicsModel& kinematicsModel,
                                                     const message::vision::Balls& fixationObjects,
                                                     const message::behaviour::SoccerObjectPriority::SearchType& sType,
                                                     const message::input::Sensors& sensors,
                                                     const message::input::Image::Lens& lens);
        std::vector<Eigen::Vector2d> getSearchPoints(const message::motion::KinematicsModel& kinematicsModel,
                                                     const message::vision::Goals& fixationObjects,
                                                     const message::behaviour::SoccerObjectPriority::SearchType& sType,
                                                     const message::input::Sensors& sensors,
                                                     const message::input::Image::Lens& lens);

        /*! @brief Combines a collection of vision objects. The screen resulting screen angular region is the
         * bounding box of the objects
         */
        message::vision::Ball combineVisionObjects(const message::vision::Balls& obs);
        message::vision::Goal combineVisionObjects(const message::vision::Goals& obs);

        /*! @brief Gets a bounding box in screen angular space of a set of vision objects
         */
        static utility::math::geometry::Quad<double, 2, 1> getScreenAngularBoundingBox(
            const message::vision::Balls& obs);
        static utility::math::geometry::Quad<double, 2, 1> getScreenAngularBoundingBox(
            const message::vision::Goals& obs);

        bool orientationHasChanged(const message::input::Sensors& sensors);


        // CONFIG - loaded elsewhere
        float max_yaw   = 0.0f;
        float min_yaw   = 0.0f;
        float max_pitch = 0.0f;
        float min_pitch = 0.0f;


        float replan_angle_threshold = 0.0f;
        Eigen::Matrix3d Rtw{};

        // CONFIG from HeadBehaviourSoccer.yaml
        float pitch_plan_threshold                = 0.0f;
        float pitch_plan_value                    = 20.0f;
        double fractional_view_padding            = 0.0;
        float search_timeout_ms                   = 0.0f;
        float fractional_angular_update_threshold = 0.0f;

        bool oscillate_search = false;

        bool locBallReceived = false;
        message::localisation::Ball lastLocBall{};

        std::map<message::behaviour::SoccerObjectPriority::SearchType, std::vector<Eigen::Vector2d>> searches{};

        // State variables
        Searcher<Eigen::Vector2d> headSearcher{};

        int ballPriority = 0;
        int goalPriority = 0;
        message::behaviour::SoccerObjectPriority::SearchType searchType =
            message::behaviour::SoccerObjectPriority::SearchType::LOST;

        NUClear::clock::time_point lastPlanUpdate{};
        NUClear::clock::time_point timeLastObjectSeen{};

        Eigen::Vector2d lastCentroid = Eigen::Vector2d::Zero();

        bool lostAndSearching = false;
        bool lostLastTime     = false;

        bool isGettingUp = false;

        int lastBallPriority = 0;
        int lastGoalPriority = 0;

    public:
        explicit HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::behaviour::skills

#endif  // MODULES_BEHAVIOURS_REFLEX_HEADBEHAVIOURSOCCER_HPP
