/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#ifndef MODULE_PLANNING_WALK_PATH_CONTROL_HPP
#define MODULE_PLANNING_WALK_PATH_CONTROL_HPP

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

namespace module::planning::walk_path {

    struct WalkToParams {
        /// @brief Maximum velocity command per axis [vx (m/s), vy (m/s), vtheta (rad/s)]
        Eigen::Vector3d max_velocity = Eigen::Vector3d::Zero();
        /// @brief Maximum backward (negative x) velocity magnitude (m/s)
        double max_backward_velocity = 0.0;
        /// @brief Translational speed per metre of distance to the target (1/s)
        double k_translation = 0.0;
        /// @brief Angular velocity per radian of heading error (1/s)
        double k_theta = 0.0;
        /// @brief Distance to target to start aligning with the final heading (m)
        double max_align_radius = 0.0;
        /// @brief Distance to target to be fully aligned with the final heading (m)
        double min_align_radius = 0.0;
        /// @brief Heading error above which the robot only rotates on the spot (rad)
        double max_angle_error = 0.0;
        /// @brief Heading error below which full forward speed is allowed (rad)
        double min_angle_error = 0.0;
    };

    struct WalkToResult {
        /// @brief Velocity command [vx, vy, vtheta] before smoothing
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        /// @brief Heading the controller is steering toward (rad)
        double desired_heading = 0.0;
    };

    /// @brief Computes a clamped progress in [0,1]: 0 when |error| >= hi, 1 when |error| <= lo
    inline double prog(const double error, const double lo, const double hi) {
        return std::clamp((hi - std::abs(error)) / (hi - lo), 0.0, 1.0);
    }

    /// @brief Linearly interpolates between start and end by t in [0,1]
    inline double lerp(const double start, const double end, const double t) {
        return start + (end - start) * t;
    }

    /// @brief Interpolates between two angles by t in [0,1] along the shortest arc, wrapping the result to [-pi, pi]
    inline double lerp_angle(const double start, const double end, const double t) {
        return std::remainder(start + std::remainder(end - start, 2.0 * M_PI) * t, 2.0 * M_PI);
    }

    /// @brief Two-regime proportional velocity controller for walking to a pose. Far from the target the robot faces
    /// it and drives forward only, slowing while turning; near the target it approaches omnidirectionally while
    /// blending its heading toward the final desired heading. Speed is proportional to distance in both regimes so
    /// the command is continuous at the boundary.
    /// @param rDRr 2D vector from robot to target in the robot frame (m)
    /// @param angle_to_final_heading Bearing of the desired final heading in the robot frame (rad)
    /// @param p Controller parameters
    inline WalkToResult walk_to_velocity(const Eigen::Vector2d& rDRr,
                                         const double angle_to_final_heading,
                                         const WalkToParams& p) {
        const double distance        = rDRr.norm();
        const double angle_to_target = std::atan2(rDRr.y(), rDRr.x());

        WalkToResult result{};

        if (distance > p.max_align_radius) {
            // Rotate on the spot when badly misaligned and slow down while turning, as the gait tracks combined
            // high vx+vtheta poorly
            const double gate = prog(angle_to_target, p.min_angle_error, p.max_angle_error);

            result.desired_heading = angle_to_target;
            result.velocity.x()    = std::min(p.k_translation * distance, p.max_velocity.x()) * gate;
        }
        else {
            // Blend from facing the target to the final heading as we close in
            const double align_progress = prog(distance, p.min_align_radius, p.max_align_radius);
            result.desired_heading      = lerp_angle(angle_to_target, angle_to_final_heading, align_progress);

            // Omnidirectional approach, decelerating to zero at the target
            const double speed = std::min(p.k_translation * distance, p.max_velocity.x());
            if (distance > 0.0) {
                result.velocity.head<2>() = speed * rDRr.normalized();
            }
        }

        result.velocity.z() =
            std::clamp(p.k_theta * result.desired_heading, -p.max_velocity.z(), p.max_velocity.z());

        return result;
    }

    /// @brief Constrains a velocity command to the per-axis limits, scaling the translational components together to
    /// preserve the direction of travel. Backward speed has its own (smaller) limit as fast backward walking is
    /// unstable.
    inline Eigen::Vector3d constrain_velocity(const Eigen::Vector3d& v,
                                              const Eigen::Vector3d& max_velocity,
                                              const double max_backward_velocity) {
        // Scale factors in each translational axis (infinity if the component is 0)
        const auto inf = std::numeric_limits<double>::infinity();
        const auto sx  = v.x() != 0.0 ? max_velocity.x() / std::abs(v.x()) : inf;
        const auto sy  = v.y() != 0.0 ? max_velocity.y() / std::abs(v.y()) : inf;

        // No scaling (s=1) unless either axis exceeds its limit
        const auto s = std::min({1.0, sx, sy});

        const auto x       = std::max(v.x() * s, -max_backward_velocity);
        const auto angular = std::clamp(v.z(), -max_velocity.z(), max_velocity.z());

        return {x, v.y() * s, angular};
    }

    /// @brief Applies the walk controller's command dead-zone per axis: commands below zero_tolerance are snapped to
    /// zero, other commands too small for the controller to respond to are bumped up to min_velocity, and everything
    /// is clamped to max_velocity.
    inline Eigen::Vector3d apply_dead_zone(const Eigen::Vector3d& v,
                                           const Eigen::Vector3d& min_velocity,
                                           const Eigen::Vector3d& zero_tolerance,
                                           const Eigen::Vector3d& max_velocity) {
        Eigen::Vector3d out = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; ++i) {
            const double u = v(i);
            if (std::abs(u) < zero_tolerance(i)) {
                out(i) = 0.0;
            }
            else if (std::abs(u) < min_velocity(i)) {
                out(i) = std::copysign(min_velocity(i), u);
            }
            else {
                out(i) = std::clamp(u, -max_velocity(i), max_velocity(i));
            }
        }
        return out;
    }

}  // namespace module::planning::walk_path

#endif  // MODULE_PLANNING_WALK_PATH_CONTROL_HPP
