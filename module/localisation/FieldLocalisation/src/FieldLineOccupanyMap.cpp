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
#include "FieldLocalisation.hpp"

namespace module::localisation {

    void FieldLocalisation::setup_fieldline_distance_map(const FieldDescription& fd) {
        // Resize map to the field dimensions
        int map_width  = (fd.dimensions.field_width + 2 * fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int map_length = (fd.dimensions.field_length + 2 * fd.dimensions.border_strip_min_width) / cfg.grid_size;
        fieldline_distance_map.resize(map_width, map_length);

        // Calculate line width in grid cells
        int line_width = fd.dimensions.line_width / cfg.grid_size;

        // Add outer field lines
        int field_x0     = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int field_y0     = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int field_width  = (fd.dimensions.field_width) / cfg.grid_size;
        int field_length = (fd.dimensions.field_length) / cfg.grid_size;
        fieldline_distance_map.add_rectangle(field_x0, field_y0, field_length, field_width, line_width);

        // Add left goal area box
        int left_goal_box_x0 = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int left_goal_box_y0 =
            (fd.dimensions.border_strip_min_width + (fd.dimensions.field_width - fd.dimensions.goal_area_width) / 2)
            / cfg.grid_size;
        int left_goal_box_width  = (fd.dimensions.goal_area_length) / cfg.grid_size;
        int left_goal_box_length = (fd.dimensions.goal_area_width) / cfg.grid_size;
        fieldline_distance_map.add_rectangle(left_goal_box_x0,
                                             left_goal_box_y0,
                                             left_goal_box_width,
                                             left_goal_box_length,
                                             line_width);

        // Add right goal area box
        int right_goal_box_x0 =
            (fd.dimensions.border_strip_min_width + fd.dimensions.field_length - fd.dimensions.goal_area_length)
            / cfg.grid_size;
        int right_goal_box_y0     = left_goal_box_y0;
        int right_goal_box_width  = left_goal_box_width;
        int right_goal_box_length = left_goal_box_length;
        fieldline_distance_map.add_rectangle(right_goal_box_x0,
                                             right_goal_box_y0,
                                             right_goal_box_width,
                                             right_goal_box_length,
                                             line_width);

        // Add left penalty area box
        if (fd.dimensions.penalty_area_length != 0 && fd.dimensions.penalty_area_width != 0) {
            int left_penalty_box_x0 = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int left_penalty_box_y0 = (fd.dimensions.border_strip_min_width
                                       + (fd.dimensions.field_width - fd.dimensions.penalty_area_width) / 2)
                                      / cfg.grid_size;
            int left_penalty_box_width  = (fd.dimensions.penalty_area_width) / cfg.grid_size;
            int left_penalty_box_length = (fd.dimensions.penalty_area_length) / cfg.grid_size;
            fieldline_distance_map.add_rectangle(left_penalty_box_x0,
                                                 left_penalty_box_y0,
                                                 left_penalty_box_length,
                                                 left_penalty_box_width,
                                                 line_width);

            // Add right penalty area box
            int right_penalty_box_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length - fd.dimensions.penalty_area_length)
                / cfg.grid_size;
            int right_penalty_box_y0     = left_penalty_box_y0;
            int right_penalty_box_width  = left_penalty_box_width;
            int right_penalty_box_length = left_penalty_box_length;
            fieldline_distance_map.add_rectangle(right_penalty_box_x0,
                                                 right_penalty_box_y0,
                                                 right_penalty_box_length,
                                                 right_penalty_box_width,
                                                 line_width);
        }

        // Add centre line
        int centre_line_x0    = (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size;
        int centre_line_y0    = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int centre_line_width = (fd.dimensions.field_width) / cfg.grid_size;
        int centre_line_length = (fd.dimensions.line_width) / cfg.grid_size;
        fieldline_distance_map.add_rectangle(centre_line_x0,
                                             centre_line_y0,
                                             centre_line_length,
                                             centre_line_width,
                                             line_width);


        if (fd.dimensions.penalty_mark_distance != 0) {
            // Add left penalty cross in centre of penalty area
            int left_penalty_cross_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.penalty_mark_distance) / cfg.grid_size;
            int left_penalty_cross_y0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_width / 2) / cfg.grid_size;
            int left_penalty_cross_width = 0.1 / cfg.grid_size;
            fieldline_distance_map.add_cross(left_penalty_cross_x0,
                                             left_penalty_cross_y0,
                                             left_penalty_cross_width,
                                             line_width);

            // Add right penalty cross in centre of penalty area
            int right_penalty_cross_x0 = (fd.dimensions.border_strip_min_width + fd.dimensions.field_length
                                          - fd.dimensions.penalty_mark_distance)
                                         / cfg.grid_size;
            int right_penalty_cross_y0    = left_penalty_cross_y0;
            int right_penalty_cross_width = 0.1 / cfg.grid_size;
            fieldline_distance_map.add_cross(right_penalty_cross_x0,
                                             right_penalty_cross_y0,
                                             right_penalty_cross_width,
                                             line_width);

            // Add centre cross in centre of field
            int centre_cross_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size
                + line_width / 2;
            int centre_cross_y0    = left_penalty_cross_y0;
            int centre_cross_width = 0.1 / cfg.grid_size;
            fieldline_distance_map.add_cross(centre_cross_x0, centre_cross_y0, centre_cross_width, line_width);
        }


        // Add centre circle
        int centre_circle_x0 = (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size;
        int centre_circle_y0 = (fd.dimensions.border_strip_min_width + fd.dimensions.field_width / 2) / cfg.grid_size;
        int centre_circle_r  = (fd.dimensions.center_circle_diameter / 2) / cfg.grid_size;
        fieldline_distance_map.add_circle(centre_circle_x0, centre_circle_y0, centre_circle_r, line_width);

        // Precompute the distance map
        fieldline_distance_map.create_distance_map(cfg.grid_size);
    }

    void FieldLocalisation::setup_field_landmarks(const FieldDescription& fd) {
        // Half dimensions for easier calculation
        double half_length = fd.dimensions.field_length / 2;
        double half_width  = fd.dimensions.field_width / 2;

        // Corners of the field
        landmarks.push_back({Eigen::Vector3d(-half_length, half_width, 0),
                             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back({Eigen::Vector3d(-half_length, -half_width, 0),
                             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back({Eigen::Vector3d(half_length, half_width, 0),
                             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back({Eigen::Vector3d(half_length, -half_width, 0),
                             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});

        // Mid-points of each sideline
        landmarks.push_back(
            {Eigen::Vector3d(0, half_width, 0), message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector3d(0, -half_width, 0), message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});

        // X intersection at the center
        landmarks.push_back(
            {Eigen::Vector3d(0, 0, 0), message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});
        landmarks.push_back({Eigen::Vector3d(0, fd.dimensions.center_circle_diameter / 2, 0),
                             message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});
        landmarks.push_back({Eigen::Vector3d(0, -fd.dimensions.center_circle_diameter / 2, 0),
                             message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});

        if (fd.dimensions.penalty_area_length != 0 && fd.dimensions.penalty_area_width != 0) {
            // T intersections from the penalty areas
            landmarks.push_back({Eigen::Vector3d(half_length, fd.dimensions.penalty_area_width / 2, 0),
                                 message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
            landmarks.push_back({Eigen::Vector3d(half_length, -fd.dimensions.penalty_area_width / 2, 0),
                                 message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
            landmarks.push_back({Eigen::Vector3d(-half_length, fd.dimensions.penalty_area_width / 2, 0),
                                 message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
            landmarks.push_back({Eigen::Vector3d(-half_length, -fd.dimensions.penalty_area_width / 2, 0),
                                 message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
        }

        // T intersections from the goal areas
        landmarks.push_back({Eigen::Vector3d(half_length, fd.dimensions.goal_area_width / 2, 0),
                             message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
        landmarks.push_back({Eigen::Vector3d(half_length, -fd.dimensions.goal_area_width / 2, 0),
                             message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
        landmarks.push_back({Eigen::Vector3d(-half_length, fd.dimensions.goal_area_width / 2, 0),
                             message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});
        landmarks.push_back({Eigen::Vector3d(-half_length, -fd.dimensions.goal_area_width / 2, 0),
                             message::vision::FieldIntersection::IntersectionType::T_INTERSECTION});

        // L intersections from the penalty areas
        landmarks.push_back(
            {Eigen::Vector3d(half_length - fd.dimensions.penalty_area_length, fd.dimensions.penalty_area_width / 2, 0),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector3d(half_length - fd.dimensions.penalty_area_length, -fd.dimensions.penalty_area_width / 2, 0),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector3d(-half_length + fd.dimensions.penalty_area_length, fd.dimensions.penalty_area_width / 2, 0),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back({Eigen::Vector3d(-half_length + fd.dimensions.penalty_area_length,
                                             -fd.dimensions.penalty_area_width / 2,
                                             0),
                             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});

        // L intersections from the goal areas
        landmarks.push_back(
            {Eigen::Vector3d(half_length - fd.dimensions.goal_area_length, fd.dimensions.goal_area_width / 2, 0),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector3d(half_length - fd.dimensions.goal_area_length, -fd.dimensions.goal_area_width / 2, 0),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector3d(-half_length + fd.dimensions.goal_area_length, fd.dimensions.goal_area_width / 2, 0),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector3d(-half_length + fd.dimensions.goal_area_length, -fd.dimensions.goal_area_width / 2, 0),
             message::vision::FieldIntersection::IntersectionType::L_INTERSECTION});

        // X intersections from penalty spots
        landmarks.push_back({Eigen::Vector3d(half_length - fd.dimensions.penalty_mark_distance, 0, 0),
                             message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});
        landmarks.push_back({Eigen::Vector3d(-half_length + fd.dimensions.penalty_mark_distance, 0, 0),
                             message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});
        landmarks.push_back(
            {Eigen::Vector3d(0, 0, 0), message::vision::FieldIntersection::IntersectionType::X_INTERSECTION});
    }

}  // namespace module::localisation
