/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#ifndef UTILITY_STRATEGY_POSITIONING_HPP
#define UTILITY_STRATEGY_POSITIONING_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <utility>
#include <vector>

#include "message/support/FieldDescription.hpp"

#include "utility/math/euler.hpp"

namespace utility::strategy {

    using message::support::FieldDescription;

    /**
     * @brief Cost function for positioning robots in a formation
     *
     * @param X The position of the robot in world coordinates
     * @param neutral_point The neutral point to be close to
     * @param opponent_positions The positions of the opponent robots
     * @param all_robot_positions The positions of all robots (including self)
     * @param clearance The clearance distance to avoid other robots
     * @param equidist_weight Weight for equidistance cost
     * @param neutral_weight Weight for closeness to neutral point
     * @param avoid_weight Weight for avoiding other robots
     *
     * @return The cost of the current position
     */
    template <typename Scalar>
    Scalar cost(const Eigen::Vector3d& X,
                const Eigen::Vector3d& neutral_point,
                const std::vector<Eigen::Vector3d>& opponent_positions,
                const std::vector<Eigen::Vector3d>& all_robot_positions,
                Scalar clearance,
                Scalar equidist_weight,
                Scalar neutral_weight,
                Scalar avoid_weight) {
        Scalar cost = 0.0;

        // Average distance to opponent robots
        Scalar av_dist_opp = 0.0;
        for (const auto& opp : opponent_positions) {
            av_dist_opp += (X.head<2>() - opp.head<2>()).norm();
        }
        av_dist_opp = av_dist_opp / opponent_positions.size();

        // Cost for not being equidistant to all opponents
        for (const auto& opp : opponent_positions) {
            Scalar d   = (X.head<2>() - opp.head<2>()).norm();
            Scalar err = d - av_dist_opp;
            cost += equidist_weight * err * err;
        }

        // Encourage closeness to the neutral point
        cost += neutral_weight * (X.head<2>() - neutral_point.head<2>()).norm();

        auto penalty = [](Scalar d, Scalar clearance) {
            if (d < clearance) {
                Scalar diff = clearance - d;
                return diff * diff;
            }
            return 0.0;
        };

        // Penalise proximity to any robot
        for (const auto& other : all_robot_positions) {
            Scalar d = (X.head<2>() - other.head<2>()).norm();
            cost += avoid_weight * penalty(d, clearance);
        }

        return cost;
    }

    /**
     * @brief Find the optimal position for a robot using gradient descent
     *
     * @param initial The initial position of the robot
     * @param neutral_point The neutral point to be close to
     * @param opponent_positions The positions of the opponent robots
     * @param all_robot_positions The positions of all robots (including self)
     * @param clearance The clearance distance to avoid other robots
     * @param equidist_weight Weight for equidistance cost
     * @param neutral_weight Weight for closeness to neutral point
     * @param avoid_weight Weight for avoiding other robots
     * @param step_size Step size for gradient descent
     * @param max_iters Maximum number of iterations for gradient descent
     *
     * @return The optimal position for the robot
     */
    template <typename Scalar>
    Eigen::Vector3d optimal_pos(const Eigen::Vector3d& initial,
                                const Eigen::Vector3d& neutral_point,
                                const std::vector<Eigen::Vector3d>& opponent_positions,
                                const std::vector<Eigen::Vector3d>& all_robot_positions,
                                const Scalar clearance,
                                const Scalar equidist_weight,
                                const Scalar neutral_weight,
                                const Scalar avoid_weight,
                                const Scalar step_size,
                                const int max_iters) {
        Eigen::Vector3d X = initial;

        // Gradient descent to find the optimal position
        for (int iter = 0; iter < max_iters; ++iter) {
            Eigen::Vector3d grad(0.0, 0.0, 0.0);

            // Numerical gradient
            Scalar eps = 1e-5;
            for (int i = 0; i < 2; ++i) {
                Eigen::Vector3d X_eps = X;
                X_eps[i] += eps;
                Scalar f1 = cost(X_eps,
                                 neutral_point,
                                 opponent_positions,
                                 all_robot_positions,
                                 clearance,
                                 equidist_weight,
                                 neutral_weight,
                                 avoid_weight);
                X_eps[i] -= 2 * eps;
                Scalar f2 = cost(X_eps,
                                 neutral_point,
                                 opponent_positions,
                                 all_robot_positions,
                                 clearance,
                                 equidist_weight,
                                 neutral_weight,
                                 avoid_weight);
                grad[i]   = (f1 - f2) / (2 * eps);
            }

            // Update the position
            X -= step_size * grad;
        }

        return X;
    }

    /**
     * @brief Find the index of a target position in a list of positions
     *
     * @param list The list of positions to search
     * @param target The target position to find
     *
     * @return The index of the target position in the list, or -1 if not found
     */
    int index_of(const std::vector<Eigen::Vector3d>& list, const Eigen::Vector3d& target) {
        auto it = std::find_if(list.begin(), list.end(), [&](const Eigen::Vector3d& p) {
            return (p - target).norm() < 1e-4;
        });
        return std::distance(list.begin(), it);
    }

    /**
     * @brief Remove a robot from a list of positions if it is close enough to the specified robot position
     *
     * @param list The list of positions to remove the robot from
     * @param robot The position of the robot to be removed
     */
    void remove_robot(std::vector<Eigen::Vector3d>& list, const Eigen::Vector3d& robot) {
        list.erase(std::remove_if(list.begin(),
                                  list.end(),
                                  [&](const Eigen::Vector3d& p) { return (p - robot).norm() < 1e-4; }),
                   list.end());
    }

    /**
     * @brief Calculate the ready position for a robot based on its teammates and field dimensions
     *
     * @param Hfw Transformation from world to field coordinates
     * @param Hrw Transformation from world to robot coordinates
     * @param teammates List of teammate positions in field coordinates
     * @param field_desc Field description containing dimensions of the field
     * @param kick_off Whether the robot is in a kick-off situation
     *
     * @return The ready position for the robot in field coordinates
     */
    Eigen::Isometry3d ready_position(const Eigen::Isometry3d& Hfw,
                                     const Eigen::Isometry3d& Hrw,
                                     const std::vector<Eigen::Vector3d>& teammates,
                                     const FieldDescription& field_desc,
                                     const bool kick_off) {
        // Transform robot position to field coordinates and get side
        Eigen::Vector3d rRFf = (Hfw * Hrw.inverse()).translation();
        bool is_left         = rRFf.y() > 0;

        // Sort teammates into left and right
        std::vector<Eigen::Vector3d> left, right;
        for (const auto& t : teammates)
            (t.y() > 0 ? left : right).push_back(t);

        // Sort out kick off side and find our side vector
        auto& side = is_left ? left : right;
        side.push_back(rRFf);
        bool left_kick_off = left.size() >= right.size();

        // Sort the side vector by x position, so that the first robot is the closest to the centre
        std::sort(side.begin(), side.end(), [](auto& a, auto& b) { return std::abs(a.x()) < std::abs(b.x()); });

        // Check if we are the kick off robot
        bool kicking_side             = (is_left && left_kick_off) || (!is_left && !left_kick_off);
        Eigen::Vector3d kickoff_robot = kicking_side ? side.front() : Eigen::Vector3d::Zero();
        bool is_kickoff_robot         = kicking_side && (rRFf - kickoff_robot).norm() < 1e-4;

        // If we are the kick off robot, we want to be in the centre circle
        double c_rad = field_desc.dimensions.center_circle_diameter / 2.0;
        if (is_kickoff_robot) {
            Eigen::Vector3d p =
                kick_off ? Eigen::Vector3d(c_rad / 2, 0.0, 0.0) : Eigen::Vector3d(c_rad + 0.3, 0.0, M_PI);
            return utility::math::euler::pos_rpy_to_transform(p, Eigen::Vector3d(0, 0, -M_PI));
        }

        // Remove the kick off robot from the side vector if it's our side
        if (kicking_side) {
            remove_robot(side, kickoff_robot);
        }

        // Make equidistant grid and find where we should be in the grid
        int index = index_of(side, rRFf);
        int rows  = static_cast<int>(side.size());

        // Equally space the robots between the kick off robot and goalie
        double penalty_x = field_desc.dimensions.field_length / 2.0 - field_desc.dimensions.penalty_mark_distance;
        double x         = c_rad + (index + 1) * (penalty_x - c_rad) / rows;
        // Space the robots out with an offset based on their index, so that robots are not directly behind each other
        double y = (is_left ? 1 : -1) * ((index + 1) * field_desc.dimensions.field_width / (4.0 * rows));

        return utility::math::euler::pos_rpy_to_transform(Eigen::Vector3d(x, y, 0.0), Eigen::Vector3d(0, 0, -M_PI));
    }


}  // namespace utility::strategy

#endif  // UTILITY_STRATEGY_POSITIONING_HPP
