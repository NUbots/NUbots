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
#include "FieldMap.hpp"

#include <stdexcept>

namespace module::localisation::srif {

    FieldMap::FieldMap(const FieldDimensions& dims) : dims(dims) {
        build();
    }

    const std::vector<Eigen::Vector3d>& FieldMap::landmarks(LandmarkType type) const {
        switch (type) {
            case LandmarkType::L_INTERSECTION: return landmarks_l_;
            case LandmarkType::T_INTERSECTION: return landmarks_t_;
            case LandmarkType::X_INTERSECTION: return landmarks_x_;
            case LandmarkType::GOAL_POST: return landmarks_goal_post_;
        }
        throw std::invalid_argument("FieldMap::landmarks: unknown LandmarkType");
    }

    void FieldMap::build() {
        landmarks_l_.clear();
        landmarks_t_.clear();
        landmarks_x_.clear();
        landmarks_goal_post_.clear();

        const double half_length = dims.field_length / 2;
        const double half_width  = dims.field_width / 2;

        // L intersections: the four corners of the field
        landmarks_l_.emplace_back(-half_length, half_width, 0);
        landmarks_l_.emplace_back(-half_length, -half_width, 0);
        landmarks_l_.emplace_back(half_length, half_width, 0);
        landmarks_l_.emplace_back(half_length, -half_width, 0);

        if (dims.penalty_area_length != 0 && dims.penalty_area_width != 0) {
            // L intersections at the corners of the penalty areas nearest the centre of the field
            landmarks_l_.emplace_back(half_length - dims.penalty_area_length, dims.penalty_area_width / 2, 0);
            landmarks_l_.emplace_back(half_length - dims.penalty_area_length, -dims.penalty_area_width / 2, 0);
            landmarks_l_.emplace_back(-half_length + dims.penalty_area_length, dims.penalty_area_width / 2, 0);
            landmarks_l_.emplace_back(-half_length + dims.penalty_area_length, -dims.penalty_area_width / 2, 0);
        }

        // L intersections at the corners of the goal areas nearest the centre of the field
        landmarks_l_.emplace_back(half_length - dims.goal_area_length, dims.goal_area_width / 2, 0);
        landmarks_l_.emplace_back(half_length - dims.goal_area_length, -dims.goal_area_width / 2, 0);
        landmarks_l_.emplace_back(-half_length + dims.goal_area_length, dims.goal_area_width / 2, 0);
        landmarks_l_.emplace_back(-half_length + dims.goal_area_length, -dims.goal_area_width / 2, 0);

        // T intersections: where the halfway line meets the touchlines
        landmarks_t_.emplace_back(0, half_width, 0);
        landmarks_t_.emplace_back(0, -half_width, 0);

        if (dims.penalty_area_length != 0 && dims.penalty_area_width != 0) {
            // T intersections where the penalty area lines meet the goal lines
            landmarks_t_.emplace_back(half_length, dims.penalty_area_width / 2, 0);
            landmarks_t_.emplace_back(half_length, -dims.penalty_area_width / 2, 0);
            landmarks_t_.emplace_back(-half_length, dims.penalty_area_width / 2, 0);
            landmarks_t_.emplace_back(-half_length, -dims.penalty_area_width / 2, 0);
        }

        // T intersections where the goal area lines meet the goal lines
        landmarks_t_.emplace_back(half_length, dims.goal_area_width / 2, 0);
        landmarks_t_.emplace_back(half_length, -dims.goal_area_width / 2, 0);
        landmarks_t_.emplace_back(-half_length, dims.goal_area_width / 2, 0);
        landmarks_t_.emplace_back(-half_length, -dims.goal_area_width / 2, 0);

        // X intersections: the centre of the field
        landmarks_x_.emplace_back(0, 0, 0);
        // Points where the centre circle crosses the halfway line
        landmarks_x_.emplace_back(0, dims.centre_circle_diameter / 2, 0);
        landmarks_x_.emplace_back(0, -dims.centre_circle_diameter / 2, 0);

        if (dims.penalty_mark_distance != 0) {
            // Penalty marks are classified as X intersections
            landmarks_x_.emplace_back(half_length - dims.penalty_mark_distance, 0, 0);
            landmarks_x_.emplace_back(-half_length + dims.penalty_mark_distance, 0, 0);
        }

        // Goal posts: the base centre of each post
        landmarks_goal_post_.emplace_back(half_length, dims.goal_width / 2, 0);
        landmarks_goal_post_.emplace_back(half_length, -dims.goal_width / 2, 0);
        landmarks_goal_post_.emplace_back(-half_length, dims.goal_width / 2, 0);
        landmarks_goal_post_.emplace_back(-half_length, -dims.goal_width / 2, 0);

        // Painted lines
        auto seg = [this](double x0, double y0, double x1, double y1) {
            line_segments_.push_back({Eigen::Vector2d(x0, y0), Eigen::Vector2d(x1, y1)});
        };

        // Boundary
        seg(-half_length, -half_width, half_length, -half_width);
        seg(-half_length, half_width, half_length, half_width);
        seg(-half_length, -half_width, -half_length, half_width);
        seg(half_length, -half_width, half_length, half_width);

        // Halfway line
        seg(0, -half_width, 0, half_width);

        // Goal and penalty areas at both ends
        for (int s : {-1, 1}) {
            const double x_goal = s * half_length;
            const double x_ga   = s * (half_length - dims.goal_area_length);
            seg(x_goal, dims.goal_area_width / 2, x_ga, dims.goal_area_width / 2);
            seg(x_goal, -dims.goal_area_width / 2, x_ga, -dims.goal_area_width / 2);
            seg(x_ga, -dims.goal_area_width / 2, x_ga, dims.goal_area_width / 2);

            const double x_pa = s * (half_length - dims.penalty_area_length);
            seg(x_goal, dims.penalty_area_width / 2, x_pa, dims.penalty_area_width / 2);
            seg(x_goal, -dims.penalty_area_width / 2, x_pa, -dims.penalty_area_width / 2);
            seg(x_pa, -dims.penalty_area_width / 2, x_pa, dims.penalty_area_width / 2);
        }

        // Centre circle
        line_circles_.push_back({Eigen::Vector2d::Zero(), dims.centre_circle_diameter / 2});
    }
}  // namespace module::localisation::srif
