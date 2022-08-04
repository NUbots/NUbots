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
#include "message/motion/WalkCommand.hpp"

#include "utility/behaviour/MotionCommand.hpp"

namespace module::behaviour::planning {

    // using namespace message;

    using message::behaviour::MotionCommand;


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
            float walk_path_planner_priority = 0;
            /// @brief Walk command velocity for walking to ball
            float forward_speed = 0;
            /// @brief Maximum angular velocity command for walking to ball
            float max_turn_speed = 0;
            /// @brief Minimum angular velocity command for walking to ball
            float min_turn_speed = 0;
            /// @brief Rotate on spot walk command angular velocity
            float rotate_speed = 0;
            /// @brief Rotate on spot walk command forward velocity
            float rotate_speed_x = 0;
            /// @brief Rotate on spot walk command side velocity
            float rotate_speed_y = 0;
            /// @brief Walk to ready walk command forward velocity
            float walk_to_ready_speed_x = 0;
            /// @brief Walk to ready walk command side velocity
            float walk_to_ready_speed_y = 0;
            /// @brief Walk to ready walk command angular velocity
            float walk_to_ready_rotation = 0;
        } cfg;

        /// @brief Stores the latest MotionCommand
        message::behaviour::MotionCommand latest_command = utility::behaviour::StandStill();

        /// @brief The id registered in the subsumption system for this module
        const size_t subsumption_id;

        /// @brief Stores the position of the last ball seen
        Eigen::Vector3f rBTt = Eigen::Vector3f(1.0, 0.0, 0.0);

        /// @brief Walk using the walk command from a direct motion command.
        void walk_directly();

        /// @brief Walk directly towards the ball relative to the robot based on the latest VisionBall ball position
        /// measurement
        void vision_walk_path();

        /// @brief Rotate on the spot
        void rotate_on_spot();

        /// @brief Configured to emit a walk command that results in robot being in desired position after the ready
        /// phase
        void walk_to_ready();

        /// @brief Updates the priority of the module by emitting an ActionPriorities message
        /// @param priority The priority used in the ActionPriorities message
        void update_priority(const float& priority);

    public:
        explicit SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::planning

#endif  // MODULES_BEHAVIOUR_PLANNERS_SIMPLEWALKPATHPLANNER_HPP
