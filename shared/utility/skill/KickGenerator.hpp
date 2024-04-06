/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_MOTION_KICKGENERATOR_HPP
#define MODULE_MOTION_KICKGENERATOR_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <nuclear>

#include "utility/input/LimbID.hpp"
#include "utility/math/euler.hpp"
#include "utility/motion/splines/Trajectory.hpp"

namespace utility::skill {

    using utility::input::LimbID;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::motion::splines::Trajectory;
    using utility::motion::splines::Waypoint;

    template <typename Scalar>
    class KickGenerator {
    public:
        /**
         * @brief Get the swing foot pose at the current time.
         * @return Trajectory of torso.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_swing_foot_pose() const {
            return swingfoot_trajectory.pose(t);
        }

        /**
         * @brief Get the torso pose object at the current time.
         * @return Pose of torso.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_torso_pose() const {
            return torso_trajectory.pose(t);
        }

        /**
         * @brief Get the left or right foot pose at the given time in the torso {t} frame.
         * @param t Time.
         * @param limb Limb ID of foot to get pose of.
         * @return Swing foot pose at time t.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_foot_pose(const LimbID& limb) const {
            // Assign the value based on the foot planted
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Htl =
                left_foot_is_planted ? get_torso_pose().inverse() : get_torso_pose().inverse() * get_swing_foot_pose();
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Htr =
                left_foot_is_planted ? get_torso_pose().inverse() * get_swing_foot_pose() : get_torso_pose().inverse();

            // Return the desired pose of the specified foot
            if (limb == LimbID::LEFT_LEG)
                return Htl;
            if (limb == LimbID::RIGHT_LEG)
                return Htr;
            throw std::runtime_error("Invalid Limb ID");
        }

        /**
         * @brief Reset kick generator.
         */
        void reset() {
            // Reset time
            t = 0;

            // Generate trajectories
            generate_trajectories();
        }

        /**
         * @brief Run an update of the walk engine, updating the time and engine state and trajectories.
         * @param dt Time step.
         * @return Engine state.
         */
        void update(const Scalar& dt, const LimbID& limb) {

            // Generate either left or right foot kick trajectory
            if (limb == LimbID::LEFT_LEG) {
                left_foot_is_planted = false;
            }
            else if (limb == LimbID::RIGHT_LEG) {
                left_foot_is_planted = true;
            }
            else {
                throw std::runtime_error("Invalid Limb ID");
            }

            generate_trajectories();

            // Check for negative time step
            if (dt <= 0.0f) {
                NUClear::log<NUClear::WARN>("dt <= 0.0f");
                return;
            }

            // Check for too long dt
            if (dt > kick_duration) {
                NUClear::log<NUClear::WARN>("dt > kick_duration", dt, kick_duration);
                return;
            }

            // Update the phase
            t += dt;

            // Clamp the time to the duration of the kick if greater or equal to the duration
            if (t >= kick_duration) {
                t = kick_duration;
            }
        }

        /**
         * @brief Getter for the current time.
         * @return Current time.
         */
        Scalar get_time() const {
            return t;
        }

        /**
         * @brief Getter for the duration of the kick.
         * @return Duration of the kick.
         */
        Scalar get_duration() const {
            return kick_duration;
        }

        /**
         * @brief Get whether or not the left foot is planted.
         * @return True if left foot is planted, false otherwise.
         */
        bool is_left_foot_planted() const {
            return left_foot_is_planted;
        }

        /**
         * @brief Add waypoints to the foot trajectory.
         */
        void add_foot_waypoint(const Waypoint<Scalar>& waypoint) {
            foot_waypoints.push_back(waypoint);
        }

        /**
         * @brief Add waypoints to the torso trajectory.
         */
        void add_torso_waypoint(const Waypoint<Scalar>& waypoint) {
            torso_waypoints.push_back(waypoint);
        }

    private:
        // ******************************** State ********************************

        /// @brief Duration of complete kick.
        Scalar kick_duration = 0.0;

        /// @brief Current time in the step cycle.
        Scalar t = 0.0;

        /// @brief Whether the left foot is planted.
        bool left_foot_is_planted = false;

        // ******************************** Trajectories ********************************

        /// @brief Foot waypoints
        std::vector<Waypoint<Scalar>> foot_waypoints{};

        /// @brief Torso waypoints
        std::vector<Waypoint<Scalar>> torso_waypoints{};

        // 6D piecewise polynomial trajectory for swing foot.
        Trajectory<Scalar> swingfoot_trajectory{};

        // 6D piecewise polynomial trajectory for torso.
        Trajectory<Scalar> torso_trajectory{};

        /**
         * @brief Generate swing foot trajectory.
         * @return Trajectory of swing foot to kick.
         */
        void generate_trajectories() {
            // Clear current trajectories
            swingfoot_trajectory.clear();
            torso_trajectory.clear();

            // Add waypoints to trajectories
            for (auto waypoint : foot_waypoints) {
                // Flip y position sign based on which foot is planted
                waypoint.position.y() *= left_foot_is_planted ? -1.0 : 1.0;
                swingfoot_trajectory.add_waypoint(waypoint);
            }

            for (auto waypoint : torso_waypoints) {
                // Flip y position sign based on which foot is planted
                waypoint.position.y() *= left_foot_is_planted ? -1.0 : 1.0;
                torso_trajectory.add_waypoint(waypoint);
            }

            kick_duration = foot_waypoints.back().time_point;

            // Assert that the first timepoint of both trajectories is 0
            if (foot_waypoints.front().time_point != 0.0 || torso_waypoints.front().time_point != 0.0) {
                throw std::runtime_error("The first waypoint of the foot and torso trajectories must start at time 0.");
            }

            // Assert that the length of the foot waypoints is the same as the length of the torso waypoints
            if (foot_waypoints.back().time_point != torso_waypoints.back().time_point) {
                throw std::runtime_error(
                    "The last waypoint of the foot and torso trajectories must have the same time point.");
            }
        }
    };
}  // namespace utility::skill
#endif
