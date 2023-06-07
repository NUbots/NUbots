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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_PLANNING_KICKPLANNER_HPP
#define MODULES_BEHAVIOUR_PLANNING_KICKPLANNER_HPP

namespace module::behaviour::planning {

    class KickPlanner : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and setup the KickPlanner reactor.
        explicit KickPlanner(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Determines if the robot should kick based on the ball position.
        /// @param ball_pos The position of the ball in torso space.
        /// @return True if the robot should kick, false otherwise.
        [[nodiscard]] bool kick_valid(const Eigen::Vector3d& ball_pos) const;

        /// @brief Stores configuration values
        struct Config {
            double max_ball_distance        = 0.0;
            double kick_corridor_width      = 0.0;
            double seconds_not_seen_limit   = 0.0;
            double kick_forward_angle_limit = 0.0;
        } cfg;

        /// @brief Whether the robot is in a play state regardless of the GameController state
        bool force_playing = false;

        /// @brief Last time the ball was detected by the vision system
        NUClear::clock::time_point ball_last_seen{std::chrono::seconds(0)};

        /// @brief Last time the ball was detected in a position where a kick would likely be successful
        NUClear::clock::time_point last_time_valid{NUClear::clock::now()};
    };
}  // namespace module::behaviour::planning


#endif
