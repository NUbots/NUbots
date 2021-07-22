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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_GEOMETRY_CIRCLE_HPP
#define UTILITY_MATH_GEOMETRY_CIRCLE_HPP

#include <Eigen/Core>
#include <utility>

#include "utility/math/geometry/Line.hpp"

namespace utility::math::geometry {

    using utility::math::geometry::LineSegment;

    template <typename Scalar, int R>
    class Arc {
    public:
        using T = typename Eigen::Matrix<Scalar, R, 1>;
        Arc() : center(T::Zero()), radius(Scalar(0)), start(Scalar(0)), end(Scalar(0)) {}
        Arc(const Arc other) : center(other.center), start(other.start), end(other.end) {}
        Arc(const T& center_, const T& start_point, const T& end_point)
            : center(center_), start(start_point), end(end_point) {}

        /**
         * @brief Sets the center, start. and end points of the arc
         * @param center_ The center point of the arc
         * @param start_point The start point of the arc
         * @param end_point The end point of the arc
         */
        void set(const T& center_, const T& start_point, const T& end_point) {
            center = center_;
            start  = start_point;
            end    = end_point;
        }

        /**
         * @brief Returns the center point of the arc
         */
        [[nodiscard]] T getCenter() const {
            return center;
        }

        /**
         * @brief Returns the start point of the arc
         */
        [[nodiscard]] T getStart() const {
            return start;
        }

        /**
         * @brief Returns the end point of the arc
         */
        [[nodiscard]] T getEnd() const {
            return end;
        }

        /**
         * @brief Get the radius of the circle
         */
        [[nodiscard]] Scalar getRadius() const {
            return (start - center).norm();
        }

        /**
         * @brief Find the closest point on the arc to the given point
         *
         * If the query point lies outside of arc then either the start or end points will be returned (whichever is
         * closest)
         *
         * @param point The point to find the closest point for
         * @return T The closest point on the arc
         */
        [[nodiscard]] T closestPoint(const T& point) {
            // Arc is actually a single point
            const Scalar l1 = (start - center).squaredNorm();
            const Scalar l2 = (end - center).squaredNorm();
            if (l1 == Scalar(0) && l2 == Scalar(0)) {
                return center;
            }
            // Arc only consists of 2 points (center and end), treat as a line segment
            if (l1 == Scalar(0)) {
                LineSegment line(center, end);
                return line.closestPoint(point);
            }
            // Arc only consists of 2 points (center and start), treat as a line segment
            if (l2 == Scalar(0)) {
                LineSegment line(center, start);
                return line.closestPoint(point);
            }

            // Make sure the query point is within the arc bounds
            const Scalar cos_arc_angle = (start - center).normalised().dot((end - center).normalised());
            const Scalar cos_theta1    = (start - center).normalised().dot((point - center).normalised());

            // Arc angle is wider than angle between start and query point
            if (cos_arc_angle < cos_theta1) {
                return getRadius() * (point - center).normalized();
            }

            // Arc angle is narrower than angle between start and query point (query point is past the end)
            if (cos_arc_angle > cos_theta1) {
                return end;
            }
            return start;
        }

        /**
         * @brief Find the distance from the given point to the closest point on the circle
         *
         * @param point The point to find the distance to
         * @return std::pair<Scalar, T> The distance to the closest point and the closest point
         */
        [[nodiscard]] std::pair<Scalar, T> distanceToPoint(const T& point) {
            T p = closestPoint(point);
            return std::make_pair((point - p).norm(), p);
        }

    private:
        /// @brief The start of the line segment
        T start;
        /// @brief The end of the line segment
        T end;
    };

    template <typename Scalar, int R>
    class Circle {
    public:
        using T = typename Eigen::Matrix<Scalar, R, 1>;
        Circle() : center(T::Zero()), radius(Scalar(0)) {}
        Circle(const Circle other) : center(other.center), radius(other.radius) {}
        Circle(const T& center_, const Scalar& radius_) : center(center_), radius(radius_) {}

        /**
         * @brief Sets the center point and radius of the circle
         * @param center_ The center point of the circle
         * @param radius_ The radius of the circle
         */
        void set(const T& center_, const Scalar& radius_) {
            center = center_;
            radius = radius_;
        }

        /**
         * @brief Returns the center point of the circle
         */
        [[nodiscard]] T getCenter() const {
            return center;
        }

        /**
         * @brief Get the radius of the circle
         */
        [[nodiscard]] Scalar getRadius() const {
            return radius;
        }

        /**
         * @brief Find the closest point on the circle to the given point
         *
         * @param point The point to find the closest point for
         * @return T The closest point on the circle
         */
        [[nodiscard]] T closestPoint(const T& point) {
            // Circle is actually a single point
            if (radius == Scalar(0)) {
                return center;
            }

            // Get the direction from the center point to the query point, then scale it out by the radius
            // This should get us the closest point on the circle
            return (point - center).normalized() * radius;
        }

        /**
         * @brief Find the distance from the given point to the closest point on the circle
         *
         * @param point The point to find the distance to
         * @return std::pair<Scalar, T> The distance to the closest point and the closest point
         */
        [[nodiscard]] std::pair<Scalar, T> distanceToPoint(const T& point) {
            T p = closestPoint(point);
            return std::make_pair((point - p).norm(), p);

        private:
            /// @brief The start of the line segment
            T start;
            /// @brief The end of the line segment
            T end;
        };

    }  // namespace utility::math::geometry

#endif  // UTILITY_MATH_GEOMETRY_CIRCLE_HPP
