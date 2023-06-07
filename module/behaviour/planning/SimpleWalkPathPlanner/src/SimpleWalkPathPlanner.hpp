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

#ifndef MODULES_BEHAVIOUR_PLANNERS_SIMPLEWALKPATHPLANNER_HPP
#define MODULES_BEHAVIOUR_PLANNERS_SIMPLEWALKPATHPLANNER_HPP

#include <Eigen/Core>
#include <cmath>
#include <nuclear>

#include "extension/Configuration.hpp"

#include "message/behaviour/MotionCommand.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/motion/WalkCommand.hpp"

#include "utility/behaviour/MotionCommand.hpp"

namespace module::behaviour::planning {

    // using namespace message;

    using message::behaviour::MotionCommand;

    using FilteredBall = message::localisation::FilteredBall;


    /**
     *
     * @author Thomas O'Brien
     *
     * Creates various walk path plans
     *
     */
    class SimpleWalkPathPlanner : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            Config() = default;
            /// @brief Walk path planner priority in the subsumption system
            double walk_path_planner_priority = 0;
            /// @brief Walk command velocity for walking to ball
            double forward_speed = 0;
            /// @brief Maximum angular velocity command for walking to ball
            double max_turn_speed = 0;
            /// @brief Minimum angular velocity command for walking to ball
            double min_turn_speed = 0;
            /// @brief Rotate on spot walk command angular velocity
            double rotate_speed = 0;
            /// @brief Rotate on spot walk command forward velocity
            double rotate_speed_x = 0;
            /// @brief Rotate on spot walk command side velocity
            double rotate_speed_y = 0;
            /// @brief Walk to ready walk command forward velocity
            double walk_to_ready_speed_x = 0;
            /// @brief Walk to ready walk command side velocity
            double walk_to_ready_speed_y = 0;
            /// @brief Walk to ready walk command angular velocity
            double walk_to_ready_rotation = 0;
            /// @brief rotate_around_ball command angular velocity
            double rotate_around_ball_speed = 0;
            /// @brief rotate_around_ball forward velocity
            double rotate_around_ball_speed_x = 0;
            /// @brief rotate_around_ball side velocity
            double rotate_around_ball_speed_y = 0;
            /// @brief ball y offset
            double ball_y_offset = 0;
        } cfg;

        /// @brief Stores the latest MotionCommand
        message::behaviour::MotionCommand latest_command = utility::behaviour::StandStill();

        /// @brief The id registered in the subsumption system for this module
        const size_t subsumption_id;

        /// @brief Walk using the walk command from a direct motion command.
        void walk_directly();

        /// @brief Walk directly towards the ball relative to the robot based on the latest VisionBall ball position
        /// measurement
        /// @param ball The latest FilteredBall message which contains the ball position
        void vision_walk_path(const std::shared_ptr<const FilteredBall>& ball);

        /// @brief Rotate on the spot
        /// @param clockwise True if rotation command clockwise, false if rotation command anticlockwise
        void rotate_on_spot(bool clockwise);

        /// @brief rotate_around_ball
        void rotate_around_ball();

        /// @brief Configured to emit a walk command that results in robot being in desired position after the ready
        /// phase
        void walk_to_ready();

        /// @brief Updates the priority of the module by emitting an ActionPriorities message
        /// @param priority The priority used in the ActionPriorities message
        void update_priority(const double& priority);

    public:
        explicit SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::planning

#endif  // MODULES_BEHAVIOUR_PLANNERS_SIMPLEWALKPATHPLANNER_HPP
