/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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

#ifndef UTILITY_MATH_GEOMETRY_INTERSECTION_HPP
#define UTILITY_MATH_GEOMETRY_INTERSECTION_HPP

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace utility::math::geometry {

    /// @brief Determines if the line between the two points given intersects the circle with given radius and centre
    /// The points are used to define the line, the line extends beyond the points given
    /// @param line_start The start point of the line
    /// @param line_end The end point of the line
    /// @param circle_centre The centre of the circle
    /// @param circle_radius The radius of the circle
    /// @return True if the line intersects the circle, false otherwise
    bool intersection_line_and_circle(const Eigen::Vector2d& line_start,
                                      const Eigen::Vector2d& line_end,
                                      const Eigen::Vector2d& circle_centre,
                                      const double circle_radius) {
        // Calculate the distance between the circle centre and the line
        const Eigen::Vector2d line_vector   = line_end - line_start;
        const Eigen::Vector2d circle_vector = circle_centre - line_start;
        const double line_length            = line_vector.norm();
        const double circle_distance        = circle_vector.norm();

        // Calculate the dot product of the line and circle vectors
        const double dot_product = line_vector.dot(circle_vector) / (line_length * circle_distance);

        // Calculate the distance between the circle centre and the line
        const double distance_to_line = circle_distance * std::sqrt(1 - dot_product * dot_product);

        // Check if the distance is less than the circle radius
        return distance_to_line <= circle_radius;
    }


}  // namespace utility::math::geometry

#endif
