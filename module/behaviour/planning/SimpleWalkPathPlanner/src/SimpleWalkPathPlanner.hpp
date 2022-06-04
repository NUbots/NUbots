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

#include "message/behaviour/KickPlan.hpp"
#include "message/behaviour/MotionCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/motion/KickCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"

namespace module::behaviour::planning {

    // using namespace message;

    using message::behaviour::KickPlan;
    using message::behaviour::MotionCommand;
    using message::behaviour::WantsToKick;
    using message::input::Sensors;

    /**
     *
     * @author Thomas O'Brien
     *
     * Creates various walk path plans
     *
     */
    class SimpleWalkPathPlanner : public NUClear::Reactor {
    private:
        struct Config {
            Config()                     = default;
            float forward_speed          = 0;
            float side_speed             = 0;
            float max_turn_speed         = 0;
            float min_turn_speed         = 0;
            float rotate_speed           = 0;
            float rotate_speed_x         = 0;
            float rotate_speed_y         = 0;
            float walk_to_ready_speed_x  = 0;
            float walk_to_ready_speed_y  = 0;
            float walk_to_ready_rotation = 0;
        } cfg;

        // Stores the latest
        message::behaviour::MotionCommand latest_command;
        const size_t subsumptionId;

        /// @brief Stores the position of the last ball seen
        Eigen::Vector3f rBTt = Eigen::Vector3f(1.0, 0.0, 0.0);

        /// @brief Walk using the latest walk commands request
        void walk_directly();

        /// @brief Walk directly towards the ball relative to the robot based on the latest VisionBall ball position
        /// measurement
        void vision_walk_path();

        /// @brief Rotate on the spot
        void rotate_on_spot();

        /// @brief Configured to emit a walk command that results in robot being in desired position after the ready
        /// phase
        void walk_to_ready();

    public:
        explicit SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::planning

#endif  // MODULES_BEHAVIOUR_PLANNERS_SIMPLEWALKPATHPLANNER_HPP
