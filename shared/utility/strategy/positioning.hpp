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
                                     const bool kick_off,
                                     const double center_offset) {
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
            Eigen::Vector3d p = kick_off ? Eigen::Vector3d(c_rad / 2 + center_offset, 0.0, 0.0)
                                         : Eigen::Vector3d(c_rad + center_offset, 0.0, 0.0);
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
