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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_GEOMETRY_CONVEXHULL_H
#define UTILITY_MATH_GEOMETRY_CONVEXHULL_H

#include <Eigen/Core>
#include <vector>

namespace utility {
namespace math {
    namespace geometry {

        // Calculates the turning direction of 3 points
        // Returns -1 indicating an anti-clockwise turn
        // Returns  0 indicating a colinear set of points (no turn)
        // Returns  1 indicating a clockwise turn
        int8_t turn_direction(const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) {
            // Compute z-coordinate of the cross product of P0->P1 and P0->P2
            float cross_z = (p1.y() - p0.y()) * (p2.x() - p1.x()) - (p1.x() - p0.x()) * (p2.y() - p1.y());

            // Clockwise turn
            if (cross_z < 0.0f) {
                return 1;
            }
            // Anti-clockwise turn
            else if (cross_z > 0.0f) {
                return -1;
            }
            // Colinear
            else {
                return 0;
            }
        }

        // A point is in the convex hull if, for every pair of points in the hull, an anti-clockwise turn is made with
        // the given point
        // If the point is allowed to be on the boundary the, for every pair of points in the convex hull, the given
        // point is allowed to be colinear
        bool point_in_convex_hull(const Eigen::Vector2f& point,
                                  const std::vector<int>& hull_indices,
                                  const Eigen::MatrixXf& coords,
                                  const bool& allow_boundary = false) {
            const int8_t threshold = (allow_boundary) ? -1 : 0;
            for (auto it = std::next(hull_indices.begin()); it != hull_indices.end(); it = std::next(it)) {
                if (turn_direction(coords.row(*std::prev(it)), coords.row(*(it)), point) > threshold) {
                    return false;
                }
            }

            return true;
        }

        // Finds the convex hull of a set of points using the Graham Scan algorithm
        // https://en.wikipedia.org/wiki/Graham_scan
        std::vector<int> graham_scan(const std::vector<int>& indices, const Eigen::MatrixXf& coords) {
            // The convex hull indices
            std::vector<int> hull_indices;

            // Make a local copy of indices so we can mutate it
            std::vector<int> local_indices(indices.begin(), indices.end());

            // Find the bottom left point
            size_t bottom_left = 0;
            for (size_t idx = 1; idx < indices.size(); ++idx) {
                const Eigen::Vector2f min_p(coords.row(local_indices[bottom_left]));
                const Eigen::Vector2f p(coords.row(local_indices[idx]));

                if ((p.y() > min_p.y()) || ((p.y() == min_p.y()) && (p.x() < min_p.x()))) {
                    bottom_left = idx;
                }
            }

            // Move bottom left to front of list
            if (bottom_left != 0) {
                std::swap(local_indices[0], local_indices[bottom_left]);
                bottom_left = 0;
            }

            // Sort points by increasing angle with respect to the bottom left point (don't include the bottom left
            // point in the sort)
            std::sort(std::next(local_indices.begin()),
                      local_indices.end(),
                      [&bottom_left, &coords](const int& a, const int& b) {
                          const Eigen::Vector2f p0(coords.row(bottom_left));
                          const Eigen::Vector2f p1(coords.row(a));
                          const Eigen::Vector2f p2(coords.row(b));

                          int direction = turn_direction(p0, p1, p2);

                          // If there is no turn then closest point should be sorted before the furthest point
                          if (direction == 0) {
                              return ((p1 - p0).squaredNorm() < (p2 - p0).squaredNorm());
                          }
                          // Otherwise, sort anti-clockwise turns before clockwise turns
                          else {
                              return (direction < 0);
                          }
                      });

            // Remove all colinear points
            const Eigen::Vector2f p0(coords.row(local_indices[bottom_left]));
            for (auto it = std::next(local_indices.begin()); it != std::prev(local_indices.end());) {
                const Eigen::Vector2f p1(coords.row(*it));
                const Eigen::Vector2f p2(coords.row(*std::next(it)));

                if (turn_direction(p0, p1, p2) == 0) {
                    it = local_indices.erase(it);
                }
                else {
                    it = std::next(it);
                }
            }

            // We need a minimum of 3 non-colinear points to calculate the convex hull
            if (local_indices.size() < 3) {
                return hull_indices;
            }

            // Add the initial points to the convex hull
            hull_indices.push_back(local_indices[0]);
            hull_indices.push_back(local_indices[1]);
            hull_indices.push_back(local_indices[2]);

            // Add first point on to the end of the list to make a complete cycle
            local_indices.push_back(local_indices[0]);

            // Now go through the rest of the points and add them to the convex hull if each triple makes an
            // anti-clockwise turn
            for (auto it = std::next(local_indices.begin(), 3); it != local_indices.end(); it = std::next(it)) {
                // Triple does not make an anti-clockwise turn, replace the last element in the list
                Eigen::Vector2f p0(coords.row(*std::prev(hull_indices.end(), 2)));
                Eigen::Vector2f p1(coords.row(*std::prev(hull_indices.end(), 1)));
                const Eigen::Vector2f p2(coords.row(*it));
                while (turn_direction(p0, p1, p2) >= 0) {
                    // Remove the offending point from the convex hull
                    hull_indices.pop_back();
                    p0 = coords.row(*std::prev(hull_indices.end(), 2));
                    p1 = coords.row(*std::prev(hull_indices.end(), 1));
                }

                // Add the new point to the convex hull
                hull_indices.push_back(*it);
            }

            return hull_indices;
        }
    }  // namespace geometry
}  // namespace math
}  // namespace utility

#endif
