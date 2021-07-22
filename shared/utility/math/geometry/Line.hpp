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

#ifndef UTILITY_MATH_GEOMETRY_LINE_HPP
#define UTILITY_MATH_GEOMETRY_LINE_HPP

#include <Eigen/Core>
#include <utility>

namespace utility::math::geometry {

    template <typename Scalar, int R, int C>
    class LineSegment {
    public:
        using T = typename Eigen::Matrix<Scalar, R, C>;
        LineSegment() : start(T::Zero()), end(T::Zero()) {}
        LineSegment(const LineSegment& other) : start(other.start), end(other.end) {}
        LineSegment(const T& start_, const T& end_) : start(start_), end(end_) {}

        /**
         * @brief Sets the start and end points of the line segment
         * @param start     The start of the line segment
         * @param end     The end of the line segment
         */
        void set(const T& start_, const T& end_) {
            start = start_;
            end   = end_;
        }

        /**
         * @brief Returns the start of the line segment
         */
        [[nodiscard]] T getStart() const {
            return start;
        }

        /**
         * @brief Get the end the line segment
         */
        [[nodiscard]] T getEnd() const {
            return end;
        }

        /**
         * @brief Find the closest point in the line segment to the given point
         *
         * @param point The point to find the closest point for
         * @return T The closest point on the line. If the point is before the start the start is returned. Similarly,
         * if it is beyond the end the end is returned.
         */
        [[nodiscard]] T closestPoint(const T& point) {
            const Scalar l2 = (start - end).squaredNorm();
            // Line segment is actually a single point
            if (l2 == Scalar(0)) {
                return start;
            }

            // Consider the line extending the segment, parameterized as start + t * (end - start).
            // We find projection of point p onto the line.
            // It falls where t = [(point - start) . (end - start)] / ||start - end||^2
            // If t is < 0 or > 1 then p is outside of the line segment
            const Scalar t = (point - start).dot(end - start) / l2;
            if (t < Scalar(0)) {
                return start;
            }
            if (t > Scalar(1)) {
                return end;
            }

            // The point is inside of the line segment, so calculate the projection of the point on to the line segment
            return start + t * (end - start);
        }

        /**
         * @brief Find the distance from the given point to the closest point on the line segment
         *
         * @details If the point is outside of the line segment then the distance to either the start or end points of
         * the segment will be calculated. Otherwise, the point is projected on to the line segment and the distance to
         * that projection is found.
         *
         * @param point The point to find the closest point for
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

}  // namespace utility::math::geometry

#endif  // UTILITY_MATH_GEOMETRY_LINE_HPP
