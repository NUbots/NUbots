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

namespace utility::strategy {

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
                const vector<Eigen::Vector3d>& opponent_positions,
                const vector<Eigen::Vector3d>& all_robot_positions,
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
        Scalar av_dist_opp = av_dist_opp / opponent_positions.size();

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

    template <typename Scalar>
    Eigen::Vector3d optimal_pos(const Eigen::Vector2d& initial,
                                const Eigen::Vector2d& neutral_point,
                                const vector<Eigen::Vector2d>& opponent_positions,
                                const vector<Eigen::Vector2d>& all_robot_positions,
                                Scalar clearance,
                                Scalar equidist_weight,
                                Scalar neutral_weight,
                                Scalar avoid_weight,
                                Scalar step_size,
                                int max_iters) {
        Eigen::Vector3d X = initial;

        for (int iter = 0; iter < max_iters; ++iter) {
            Eigen::Vector3d grad(0.0, 0.0);

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


}  // namespace utility::strategy

#endif  // UTILITY_STRATEGY_POSITIONING_HPP
