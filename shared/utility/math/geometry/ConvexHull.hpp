/*
 * MIT License
 *
 * Copyright (c) 2019 NUbots
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

#ifndef UTILITY_MATH_GEOMETRY_CONVEXHULL_HPP
#define UTILITY_MATH_GEOMETRY_CONVEXHULL_HPP

#include <Eigen/Core>
#include <vector>

namespace utility::math::geometry {

    // Convert a vec3 to a vec2
    // vec2.x = vec3.x / vec3.z
    // vec2.y = vec3.y / vec3.y
    inline Eigen::Vector2f project_vector(const Eigen::Vector3f& v) {
        return Eigen::Vector2f(v.x() / v.z(), v.y() / v.z());
    }

    // Sort a list of indices by increasing theta order
    template <typename Iterator>
    void sort_by_theta(Iterator first,
                       Iterator last,
                       const Eigen::Matrix<float, 3, Eigen::Dynamic>& rays,
                       const float& world_offset) {
        std::sort(first, last, [&](const int& a, const int& b) {
            const Eigen::Vector3f& p0(rays.col(a));
            const Eigen::Vector3f& p1(rays.col(b));

            float theta0 = std::fmod(std::atan2(p0.y(), p0.x()) + M_PI - world_offset, static_cast<float>(2.0 * M_PI));
            float theta1 = std::fmod(std::atan2(p1.y(), p1.x()) + M_PI - world_offset, static_cast<float>(2.0 * M_PI));
            return theta0 < theta1;
        });
    }

    // Calculates the turning direction of 3 points
    // Returns -1 indicating an anti-clockwise turn
    // Returns  0 indicating a colinear set of points (no turn)
    // Returns  1 indicating a clockwise turn
    template <int N>
    int8_t turn_direction(const Eigen::Matrix<float, N, 1>& p0,
                          const Eigen::Matrix<float, N, 1>& p1,
                          const Eigen::Matrix<float, N, 1>& p2) {
        // Compute z-coordinate of the cross product of P0->P1 and P0->P2
        float cross_z = (p1.y() - p0.y()) * (p2.x() - p1.x()) - (p1.x() - p0.x()) * (p2.y() - p1.y());

        // Clockwise turn
        if (cross_z < 0.0f) {
            return 1;
        }
        // Anti-clockwise turn
        if (cross_z > 0.0f) {
            return -1;
        }
        // Colinear
        return 0;
    }

    // A point is in the convex hull if, for every pair of points in the hull, an anti-clockwise turn is made
    // with the given point If the point is allowed to be on the boundary the, for every pair of points in the
    // convex hull, the given point is allowed to be colinear
    inline bool point_in_convex_hull(const Eigen::Vector2f& point,
                                     const std::vector<int>& hull_indices,
                                     const Eigen::Matrix<float, Eigen::Dynamic, 2>& coords,
                                     const bool& allow_boundary = false) {
        const int8_t threshold = (allow_boundary) ? -1 : 0;
        for (auto it = std::next(hull_indices.begin()); it != hull_indices.end(); it = std::next(it)) {
            Eigen::Vector2f p0(coords.col(*std::prev(it)));
            Eigen::Vector2f p1(coords.col(*it));
            if (turn_direction(p0, p1, point) > threshold) {
                return false;
            }
        }

        return true;
    }

    template <typename Iterator>
    inline bool point_under_hull(const Eigen::Vector3f& point,
                                 const Iterator rays_begin,
                                 const Iterator rays_end,
                                 const bool& allow_boundary = false) {
        const float theta = std::atan2(point.y(), point.x());
        auto lower_it = std::lower_bound(rays_begin, rays_end, theta, [&](const Eigen::Vector3f& p0, const float& b) {
            return std::atan2(p0.y(), p0.x()) < b;
        });
        auto upper_it = std::upper_bound(rays_begin, rays_end, theta, [&](const float& b, const Eigen::Vector3f& p0) {
            return b < std::atan2(p0.y(), p0.x());
        });

        if ((lower_it == rays_end) || (upper_it == rays_end)) {
            return false;
        }
        // lower_bound returns the first ray that has a theta value that is >= our point_theta value
        // taking the ray immediately before the lower_bound should give us a ray with theta < point_theta
        // upper_bound returns the first ray that has a theta value that is > our point_theta value
        const Eigen::Vector2f p0 = project_vector(lower_it == rays_begin ? *lower_it : *std::prev(lower_it));
        const Eigen::Vector2f p1 = project_vector(point);
        const Eigen::Vector2f p2 = project_vector(*upper_it);

        // Point should make a clockwise turn if it is under the convex hull.
        // It should be colinear if it is on the convex hull
        const int threshold = allow_boundary ? -1 : 0;
        return (turn_direction(p0, p1, p2) > threshold);
    }

    inline std::vector<int> upper_convex_hull(const std::vector<int>& indices,
                                              const Eigen::Matrix<float, Eigen::Dynamic, 2>& coords,
                                              const bool& cycle = false) {
        // We need a minimum of 3 non-colinear points to calculate the convex hull
        if (indices.size() < 3) {
            return std::vector<int>();
        }

        // The convex hull indices
        std::vector<int> hull_indices;

        // Make a local copy of indices so we can mutate it
        std::vector<int> local_indices(indices.begin(), indices.end());

        // Sort by increasing x, then by decreasing y
        std::sort(local_indices.begin(), local_indices.end(), [&coords](const int& a, const int& b) {
            const Eigen::Vector2f p0(coords.col(a));
            const Eigen::Vector2f p1(coords.col(b));

            return ((p0.x() < p1.x()) || ((p0.x() == p1.x()) && (p0.y() > p1.y())));
        });

        // Remove all colinear points
        for (auto it = std::next(local_indices.begin(), 2); it != local_indices.end();) {
            const Eigen::Vector2f p0(coords.col(*std::prev(it, 2)));
            const Eigen::Vector2f p1(coords.col(*std::prev(it, 1)));
            Eigen::Vector2f p2(coords.col(*it));

            while (turn_direction(p0, p1, p2) == 0) {
                it = local_indices.erase(it);
                p2 = coords.col(*it);
            }
            it = std::next(it);
        }

        // We need a minimum of 3 non-colinear points to calculate the convex hull
        if (local_indices.size() < 3) {
            return std::vector<int>();
        }

        // Add the initial points to the convex hull
        hull_indices.push_back(local_indices[0]);
        hull_indices.push_back(local_indices[1]);

        // Add first point on to the end of the list to make a complete cycle
        if (cycle) {
            local_indices.push_back(local_indices[0]);
        }

        // Now go through the rest of the points and add them to the convex hull if each triple makes an
        // clockwise turn
        for (auto it = std::next(local_indices.begin(), 2); it != local_indices.end(); it = std::next(it)) {
            // Triple does not make an clockwise turn, replace the last element in the list
            Eigen::Vector2f p0(coords.col(*std::prev(hull_indices.end(), 2)));
            Eigen::Vector2f p1(coords.col(*std::prev(hull_indices.end(), 1)));
            const Eigen::Vector2f p2(coords.col(*it));
            while ((hull_indices.size() > 1) && (turn_direction(p0, p1, p2) <= 0)) {
                // Remove the offending point from the convex hull
                hull_indices.pop_back();
                p0 = coords.col(*std::prev(hull_indices.end(), 2));
                p1 = coords.col(*std::prev(hull_indices.end(), 1));
            }

            // Add the new point to the convex hull
            hull_indices.push_back(*it);
        }

        return hull_indices;
    }

    inline std::vector<int> upper_convex_hull(const std::vector<int>& indices,
                                              const Eigen::Matrix<float, 3, Eigen::Dynamic>& rays,
                                              const float world_offset,
                                              const bool& cycle = false) {
        // We need a minimum of 3 non-colinear points to calculate the convex hull
        if (rays.cols() < 3) {
            return std::vector<int>();
        }

        // The convex hull indices
        std::vector<int> hull_indices;

        std::vector<int> local_indices(indices.begin(), indices.end());

        // Sort by increasing theta
        sort_by_theta(local_indices.begin(), local_indices.end(), rays, world_offset);

        // Remove all colinear points
        for (auto it = std::next(local_indices.begin(), 2); it != local_indices.end();) {
            const Eigen::Vector2f p0 = project_vector(rays.col(*std::prev(local_indices.end(), 2)));
            const Eigen::Vector2f p1 = project_vector(rays.col(*std::prev(local_indices.end(), 1)));
            Eigen::Vector2f p2       = project_vector(rays.col(*it));

            while ((it != local_indices.end()) && turn_direction(p0, p1, p2) == 0) {
                it = local_indices.erase(it);
                p2 = project_vector(rays.col(*it));
            }
            if (it != local_indices.end()) {
                it = std::next(it);
            }
        }

        // We need a minimum of 3 non-colinear points to calculate the convex hull
        if (local_indices.size() < 3) {
            return std::vector<int>();
        }

        // Add the initial points to the convex hull
        hull_indices.push_back(local_indices[0]);
        hull_indices.push_back(local_indices[1]);

        // Add first point on to the end of the list to make a complete cycle
        if (cycle) {
            local_indices.push_back(local_indices[0]);
        }

        // Now go through the rest of the points and add them to the convex hull if each triple makes an
        // clockwise turn
        for (auto it = std::next(local_indices.begin(), 2); it != local_indices.end(); it = std::next(it)) {
            Eigen::Vector2f p0       = project_vector(rays.col(*std::prev(hull_indices.end(), 2)));
            Eigen::Vector2f p1       = project_vector(rays.col(*std::prev(hull_indices.end(), 1)));
            const Eigen::Vector2f p2 = project_vector(rays.col(*it));
            // Triple does not make an clockwise turn, replace the last element in the list
            while ((hull_indices.size() > 2) && (turn_direction(p0, p1, p2) <= 0)) {
                // Remove the offending point from the convex hull
                hull_indices.pop_back();
                p0 = project_vector(rays.col(*std::prev(hull_indices.end(), 2)));
                p1 = project_vector(rays.col(*std::prev(hull_indices.end(), 1)));
            }

            // Add the new point to the convex hull
            hull_indices.push_back(*it);
        }

        return hull_indices;
    }

    // Finds the convex hull of a set of points using the Graham Scan algorithm
    // https://en.wikipedia.org/wiki/Graham_scan
    inline std::vector<int> graham_scan(const std::vector<int>& indices,
                                        const Eigen::Matrix<float, 2, Eigen::Dynamic>& coords,
                                        const bool& cycle = false) {

        // We need a minimum of 3 non-colinear points to calculate the convex hull
        if (indices.size() < 3) {
            return std::vector<int>();
        }

        // The convex hull indices
        std::vector<int> hull_indices;

        // Make a local copy of indices so we can mutate it
        std::vector<int> local_indices(indices.begin(), indices.end());

        // Find the bottom left point
        size_t bottom_left = 0;
        for (size_t idx = 1; idx < indices.size(); ++idx) {
            const Eigen::Vector2f min_p(coords.col(local_indices[bottom_left]));
            const Eigen::Vector2f p(coords.col(local_indices[idx]));

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
                      const Eigen::Vector2f p0(coords.col(bottom_left));
                      const Eigen::Vector2f p1(coords.col(a));
                      const Eigen::Vector2f p2(coords.col(b));

                      int direction = turn_direction(p0, p1, p2);

                      // If there is no turn then closest point should be sorted before the furthest point
                      if (direction == 0) {
                          return ((p1 - p0).squaredNorm() < (p2 - p0).squaredNorm());
                      }
                      // Otherwise, sort anti-clockwise turns before clockwise turns
                      return (direction < 0);
                  });

        // Remove all colinear points
        for (auto it = std::next(local_indices.begin(), 2); it != local_indices.end();) {
            const Eigen::Vector2f p0(coords.col(*std::prev(it, 2)));
            const Eigen::Vector2f p1(coords.col(*std::prev(it, 1)));
            Eigen::Vector2f p2(coords.col(*it));

            while (turn_direction(p0, p1, p2) == 0) {
                it = local_indices.erase(it);
                Eigen::Vector2f p2(coords.col(*it));
            }
            it = std::next(it);
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
        if (cycle) {
            local_indices.push_back(local_indices[0]);
        }

        // Now go through the rest of the points and add them to the convex hull if each triple makes an
        // anti-clockwise turn
        for (auto it = std::next(local_indices.begin(), 3); it != local_indices.end(); it = std::next(it)) {
            // Triple does not make an anti-clockwise turn, replace the last element in the list
            Eigen::Vector2f p0(coords.col(*std::prev(hull_indices.end(), 2)));
            Eigen::Vector2f p1(coords.col(*std::prev(hull_indices.end(), 1)));
            const Eigen::Vector2f p2(coords.col(*it));
            while (turn_direction(p0, p1, p2) >= 0) {
                // Remove the offending point from the convex hull
                hull_indices.pop_back();
                p0 = coords.col(*std::prev(hull_indices.end(), 2));
                p1 = coords.col(*std::prev(hull_indices.end(), 1));
            }

            // Add the new point to the convex hull
            hull_indices.push_back(*it);
        }

        return hull_indices;
    }
}  // namespace utility::math::geometry
#endif
