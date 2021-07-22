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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_FIELDLINESAMPLING_HPP
#define MODULES_LOCALISATION_FIELDLINESAMPLING_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <nuclear>
#include <random>
#include <string>
#include <vector>

#include "message/support/FieldDescription.hpp"

#include "utility/math/geometry/Circle.hpp"
#include "utility/math/geometry/Line.hpp"

namespace module::localisation {

    using message::support::FieldDescription;

    using utility::math::geometry::Circle;
    using utility::math::geometry::LineSegment;

    template <typename Scalar>
    [[nodiscard]] inline Circle<Scalar, 3> get_center_circle(const FieldDescription& fd) {
        return Circle(Eigen::Matrix<Scalar, 3, 1>::Zero(), fd.dimensions.center_circle_diameter * Scalar(0.5));
    }

    template <typename Scalar>
    [[nodiscard]] inline std::map<std::string, std::vector<LineSegment<Scalar, 3, 1>>> get_field_lines(
        const FieldDescription& fd) {
        // Cache needed field dimension to improve readability
        const Scalar field_length           = fd.dimensions.field_length;
        const Scalar field_width            = fd.dimensions.field_width;
        const Scalar half_field_length      = fd.dimensions.field_length * Scalar(0.5);
        const Scalar half_field_width       = fd.dimensions.field_width * Scalar(0.5);
        const Scalar line_width             = fd.dimensions.line_width;
        const Scalar half_line_width        = fd.dimensions.line_width * Scalar(0.5);
        const Scalar goal_length            = fd.dimensions.goal_area_length;
        const Scalar goal_width             = fd.dimensions.goal_area_width;
        const Scalar penalty_length         = fd.dimensions.penalty_area_length;
        const Scalar penalty_width          = fd.dimensions.penalty_area_width;
        const Scalar penalty_mark           = fd.dimensions.penalty_mark_distance;
        const Scalar center_circle_diameter = fd.dimensions.center_circle_diameter;

        std::map<std::string, std::vector<LineSegment<Scalar, 3, 1>>> field_lines;

        field_lines["opp_goal_line"] = {
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(half_field_length, -half_field_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(half_field_length, half_field_width, Scalar(0)))};
        field_lines["mid_line"] = {LineSegment(Eigen::Matrix<Scalar, 3, 1>(Scalar(0), -half_field_width, Scalar(0)),
                                               Eigen::Matrix<Scalar, 3, 1>(Scalar(0), half_field_width, Scalar(0)))};
        field_lines["own_goal_line"] = {
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(-half_field_length, -half_field_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(-half_field_length, half_field_width, Scalar(0)))};
        field_lines["left_touch_line"] = {
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(-half_field_length, -half_field_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(half_field_length, -half_field_width, Scalar(0)))};
        field_lines["right_touch_line"] = {
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(-half_field_length, half_field_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(half_field_length, half_field_width, Scalar(0)))};
        field_lines["opp_penalty_mark"] = {
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(half_field_length - penalty_mark, -line_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(half_field_length - penalty_mark, line_width, Scalar(0))),
            LineSegment(
                Eigen::Matrix<Scalar, 3, 1>(half_field_length - penalty_mark - line_width, Scalar(0), Scalar(0)),
                Eigen::Matrix<Scalar, 3, 1>(half_field_length - penalty_mark + line_width, Scalar(0), Scalar(0)))};
        field_lines["center_mark"] = {LineSegment(Eigen::Matrix<Scalar, 3, 1>(-line_width, line_width, Scalar(0)))};
        field_lines["own_penalty_mark"] = {
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(-half_field_length + penalty_mark, -line_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(-half_field_length + penalty_mark, line_width, Scalar(0))),
            LineSegment(
                Eigen::Matrix<Scalar, 3, 1>(-half_field_length + penalty_mark - line_width, Scalar(0), Scalar(0)),
                Eigen::Matrix<Scalar, 3, 1>(-half_field_length + penalty_mark + line_width, Scalar(0), Scalar(0)))};
        field_lines["opp_goal_box"] = {
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(half_field_length, -half_goal_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(half_field_length - goal_length, -half_goal_width, Scalar(0))),
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(half_field_length, half_goal_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(half_field_length - goal_length, half_goal_width, Scalar(0))),
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(half_field_length - goal_length, -half_goal_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(half_field_length - goal_length, half_goal_width, Scalar(0)))};
        field_lines["own_goal_box"] = {
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(-half_field_length, -half_goal_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(-half_field_length + goal_length, -half_goal_width, Scalar(0))),
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(-half_field_length, half_goal_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(-half_field_length + goal_length, half_goal_width, Scalar(0))),
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(-half_field_length + goal_length, -half_goal_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(-half_field_length + goal_length, half_goal_width, Scalar(0)))};
        field_lines["opp_penalty_box"] = {
            LineSegment(
                Eigen::Matrix<Scalar, 3, 1>(half_field_length, -half_penalty_width, Scalar(0)),
                Eigen::Matrix<Scalar, 3, 1>(half_field_length - penalty_length, -half_penalty_width, Scalar(0))),
            LineSegment(Eigen::Matrix<Scalar, 3, 1>(half_field_length, half_penalty_width, Scalar(0)),
                        Eigen::Matrix<Scalar, 3, 1>(half_field_length - penalty_length, half_penalty_width, Scalar(0))),
            LineSegment(
                Eigen::Matrix<Scalar, 3, 1>(half_field_length - penalty_length, -half_penalty_width, Scalar(0)),
                Eigen::Matrix<Scalar, 3, 1>(half_field_length - penalty_length, half_penalty_width, Scalar(0)))};
        field_lines["own_penalty_box"] = {
            LineSegment(
                Eigen::Matrix<Scalar, 3, 1>(-half_field_length, -half_penalty_width, Scalar(0)),
                Eigen::Matrix<Scalar, 3, 1>(-half_field_length + penalty_length, -half_penalty_width, Scalar(0))),
            LineSegment(
                Eigen::Matrix<Scalar, 3, 1>(-half_field_length, half_penalty_width, Scalar(0)),
                Eigen::Matrix<Scalar, 3, 1>(-half_field_length + penalty_length, half_penalty_width, Scalar(0))),
            LineSegment(
                Eigen::Matrix<Scalar, 3, 1>(-half_field_length + penalty_length, -half_penalty_width, Scalar(0)),
                Eigen::Matrix<Scalar, 3, 1>(-half_field_length + penalty_length, half_penalty_width, Scalar(0)))};

        return field_lines;
    }

    template <typename Scalar>
    [[nodiscard]] inline std::map<std::string, Eigen::Matrix<Scalar, Eigen::Dynamic, 3>> sample_field_points(
        const FieldDescription& fd,
        const Scalar& point_density) {

        // Cache needed field dimension to improve readability
        const Scalar field_length           = fd.dimensions.field_length;
        const Scalar field_width            = fd.dimensions.field_width;
        const Scalar half_field_length      = fd.dimensions.field_length * Scalar(0.5);
        const Scalar half_field_width       = fd.dimensions.field_width * Scalar(0.5);
        const Scalar line_width             = fd.dimensions.line_width;
        const Scalar half_line_width        = fd.dimensions.line_width * Scalar(0.5);
        const Scalar goal_length            = fd.dimensions.goal_area_length;
        const Scalar goal_width             = fd.dimensions.goal_area_width;
        const Scalar penalty_length         = fd.dimensions.penalty_area_length;
        const Scalar penalty_width          = fd.dimensions.penalty_area_width;
        const Scalar penalty_mark           = fd.dimensions.penalty_mark_distance;
        const Scalar center_circle_diameter = fd.dimensions.center_circle_diameter;

        // We want to uniformly sample points inside the field line width
        std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<Scalar> dis(-half_line_width, half_line_width);

        // Total length of all field lines on the field in metres
        const Scalar field_line_length = field_length * 2                       // Left and right lines
                                         + field_width * 3                      // Top, mid-field, and bottom lines
                                         + line_width * 10                      // Top and bottom penalty marks
                                         + line_width * 2                       // Center mark
                                         + goal_length * 4                      // Top and bottom goal areas
                                         + goal_width * 2 + penalty_length * 4  // Top and bottom penalty areas
                                         + penalty_width * 2
                                         + center_circle_diameter * M_PI;  // Center circle circumference

        // Number of meters per sampled point
        const Scalar specific_volume = Scalar(1) / point_density;

        std::map<std::string, Eigen::Matrix<Scalar, Eigen::Dynamic, 3>> rLFf;

        // Sample all of field points on the field lines
        // Top line
        // Bottom line
        // Mid-field line
        int num_points        = field_width * point_density;
        rLFf["opp_goal_line"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        rLFf["mid_line"]      = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        rLFf["own_goal_line"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        for (int point = 0; point < num_points; ++point) {
            rLFf["opp_goal_line"].row(point) << dis(gen) + half_field_length,
                -half_field_width + point * specific_volume, Scalar(0);
            rLFf["mid_line"].row(point) << dis(gen), -half_field_width + point * specific_volume, Scalar(0);
            rLFf["own_goal_line"].row(point) << dis(gen) - half_field_length,
                -half_field_width + point * specific_volume, Scalar(0);
        }

        // Left line
        // Right line
        num_points               = field_length * point_density;
        rLFf["left_touch_line"]  = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        rLFf["right_touch_line"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        for (int point = 0; point < num_points; ++point) {
            rLFf["left_touch_line"].row(point) << -half_field_length + point * specific_volume,
                dis(gen) + half_field_width, Scalar(0);
            rLFf["right_touch_line"].row(point) << -half_field_length + point * specific_volume,
                dis(gen) - half_field_width, Scalar(0);
        }

        // Center circle
        num_points            = M_PI * center_circle_diameter * point_density;
        rLFf["center_circle"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        for (int point = 0; point < num_points; ++point) {
            // Calculate point in polar coordinates
            // Evenly divide the circle into num_points wedges
            const Scalar radius = dis(gen) + center_circle_diameter * Scalar(0.5);
            const Scalar theta  = 2.0 * M_PI * point / num_points;

            // Convert to cartesian coordinates
            rLFf["center_circle"].row(point) << radius * std::cos(theta), radius * std::sin(theta), Scalar(0);
        }

        // Top penalty mark side to side
        // Bottom penalty mark side to side
        num_points               = line_width * 3 * point_density;
        rLFf["opp_penalty_mark"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        rLFf["own_penalty_mark"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        for (int point = 0; point < num_points; ++point) {
            rLFf["opp_penalty_mark"].row(point) << half_field_length - penalty_mark + dis(gen),
                line_width - point * specific_volume, Scalar(0);
            rLFf["own_penalty_mark"].row(point) << -half_field_length + penalty_mark + dis(gen),
                line_width - point * specific_volume, Scalar(0);
        }

        // Top penalty mark end to end
        // Center mark end to end
        // Bottom penalty mark end to end
        int old_num_points = num_points;
        num_points         = line_width * 3 * point_density;
        rLFf["opp_penalty_mark"].conservativeResize(old_num_points + num_points, 3);
        rLFf["center_mark"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        rLFf["own_penalty_mark"].conservativeResize(old_num_points + num_points, 3);
        for (int point = 0; point < num_points; ++point) {
            rLFf["opp_penalty_mark"].row(old_num_points + point)
                << half_field_length - penalty_mark - line_width + point * specific_volume,
                dis(gen), Scalar(0);
            rLFf["center_mark"].row(point) << -line_width + point * specific_volume, dis(gen), Scalar(0);
            rLFf["own_penalty_mark"].row(old_num_points + point)
                << -half_field_length + penalty_mark - line_width + point * specific_volume,
                dis(gen), Scalar(0);
        }

        // Top goal box left and right lines
        // Bottom goal box left and right lines
        num_points           = goal_length * point_density;
        rLFf["opp_goal_box"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        rLFf["own_goal_box"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        for (int point = 0; point < 2 * num_points; point += 2) {
            rLFf["opp_goal_box"].row(point) << half_field_length - goal_length * point * specific_volume,
                goal_width * Scalar(0.5) + dis(gen), Scalar(0);
            rLFf["opp_goal_box"].row(point + 1) << half_field_length - goal_length * point * specific_volume,
                -goal_width * Scalar(0.5) + dis(gen), Scalar(0);

            rLFf["own_goal_box"].row(point) << -half_field_length + goal_length * point * specific_volume,
                goal_width * Scalar(0.5) + dis(gen), Scalar(0);
            rLFf["own_goal_box"].row(point + 1) << -half_field_length + goal_length * point * specific_volume,
                -goal_width * Scalar(0.5) + dis(gen), Scalar(0);
        }

        // Top goal box top/bottom lines
        // Bottom goal box top/bottom lines
        old_num_points = num_points;
        num_points     = goal_width * point_density;
        rLFf["opp_goal_box"].conservativeResize(old_num_points + num_points, 3);
        rLFf["own_goal_box"].conservativeResize(old_num_points + num_points, 3);
        for (int point = 0; point < num_points; ++point) {
            rLFf["opp_goal_box"].row(old_num_points + point) << half_field_length - goal_length + dis(gen),
                -goal_width * Scalar(0.5) + point * specific_volume, Scalar(0);
            rLFf["own_goal_box"].row(old_num_points + point) << -half_field_length + goal_length + dis(gen),
                -goal_width * Scalar(0.5) + point * specific_volume, Scalar(0);
        }

        // Top penalty box left and right lines
        // Bottom penalty box left and right lines
        num_points              = penalty_length * point_density;
        rLFf["opp_penalty_box"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        rLFf["own_penalty_box"] = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(num_points, 3);
        for (int point = 0; point < 2 * num_points; point += 2) {
            rLFf["opp_penalty_box"].row(point) << half_field_length - penalty_length + point * specific_volume,
                penalty_width * Scalar(0.5) + dis(gen), Scalar(0);
            rLFf["opp_penalty_box"].row(point + 1) << half_field_length - penalty_length + point * specific_volume,
                -penalty_width * Scalar(0.5) + dis(gen), Scalar(0);

            rLFf["own_penalty_box"].row(point) << -half_field_length + point * specific_volume,
                penalty_width * Scalar(0.5) + dis(gen), Scalar(0);
            rLFf["own_penalty_box"].row(point + 1) << -half_field_length + point * specific_volume,
                -penalty_width * Scalar(0.5) + dis(gen), Scalar(0);
        }

        // Top penalty box top/bottom lines
        // Bottom penalty box top/bottom lines
        old_num_points = num_points;
        num_points     = penalty_width * point_density;
        rLFf["opp_penalty_box"].conservativeResize(old_num_points + num_points, 3);
        rLFf["own_penalty_box"].conservativeResize(old_num_points + num_points, 3);
        for (int point = 0; point < num_points; ++point) {
            rLFf["opp_penalty_box"].row(old_num_points + point) << half_field_length - penalty_length + dis(gen),
                -penalty_width * Scalar(0.5) + point * specific_volume, Scalar(0);
            rLFf["opp_penalty_box"].row(old_num_points + point) << -half_field_length + penalty_length + dis(gen),
                -penalty_width * Scalar(0.5) + point * specific_volume, Scalar(0);
        }

        return rLFf;
    }

}  // namespace module::localisation
#endif
