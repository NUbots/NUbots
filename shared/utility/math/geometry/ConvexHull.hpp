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
#include <Eigen/Dense>
#include <algorithm>
#include <stack>
#include <utility>
#include <vector>

namespace utility::math::geometry {

    enum Direction {
        COLLINEAR        = 0,
        CLOCKWISE        = 1,
        COUNTERCLOCKWISE = 2,
    };

    /// @brief Finds the order of the ordered triplet (p,q,r)
    /// Either collinear if they form a line, otherwise clockwise or counterclockwise
    /// @param p The first ordered point in our space
    /// @param q The second ordered point in our space
    /// @param r The third ordered point in our space
    /// @return zero if the points are collinear, one if clockwise, two if counterclockwise
    Direction direction(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) {
        // Find the cross product of vectors (p1, p0) x (p2, p0)
        double val = (q(1) - p(1)) * (r(0) - q(0)) - (q(0) - p(0)) * (r(1) - q(1));

        // If val is zero, return zero for collinear
        // If val is greater than zero, return one for clockwise
        // If val is less than zero, return two for counterclockwise
        return (val == 0) ? Direction::COLLINEAR : ((val > 0) ? Direction::CLOCKWISE : Direction::COUNTERCLOCKWISE);
    }

    /// @brief Sorts the points in `indices` by polar angle with the first point in `indices`
    /// @param indices Indices corresponding to points in `points` that we want to sort
    /// @param points All points in our space, including points not to be used in sorting
    void sort_polar(std::vector<int>& indices, const Eigen::Matrix<double, 2, Eigen::Dynamic>& points) {
        std::sort(indices.begin() + 1, indices.end(), [&](int i, int j) {
            double dx1 = points(0, i) - points(0, indices[0]);
            double dy1 = points(1, i) - points(1, indices[0]);
            double dx2 = points(0, j) - points(0, indices[0]);
            double dy2 = points(1, j) - points(1, indices[0]);

            double det = dx1 * dy2 - dy1 * dx2;

            if (det == 0) {
                // points p1 and p2 are on the same line from the pivot, check which one is closer
                double d1 = dx1 * dx1 + dy1 * dy1;
                double d2 = dx2 * dx2 + dy2 * dy2;
                return d1 > d2;  // keep the point that is farther
            }

            return det > 0;
        });
    }

    /// @brief Finds if p0 has a lower y value than p1 - break the tie with lower x if equal
    /// @param p0 A point, if this point is lower than p1 then return true
    /// @param p1 The point to compare p0 to
    /// @return True if p0 has a lower y value than p1, or if equal then if p0 has a lower x value
    bool lower_point(Eigen::Vector2d p0, Eigen::Vector2d p1) {
        // Find the boolean comparisons to then use in the final return
        bool y_less  = p0.y() < p1.y();
        bool y_equal = p0.y() == p1.y();
        bool x_less  = p0.x() < p1.x();
        // Return if either p0 has lower y, or if equal p0 has lower x
        return y_less || (y_equal && x_less);
    }

    /// @brief Creates a convex hull from the points in `indices` using the Graham Scan algorithm for convex hulls
    /// @param indices Indices corresponding to points in `points` that we want to use to create a convex hull
    /// @param points All points in our space, including points not to be used in the convex hull algorithm
    /// @return Indicies corresponding to points in `points` that make up the convex hull for this set of points
    std::vector<int> graham_scan(const std::vector<int>& indices,
                                 const Eigen::Matrix<double, 2, Eigen::Dynamic>& points) {
        if (indices.size() < 3) {
            return indices;
        }

        // Copy `indices` so we can modify it
        std::vector<int> indices_copy = indices;

        // Find the point with the lowest y-coordinate
        int p0 = 0;
        // Start at 1, don't need to compare p0 to itself
        for (size_t i = 1; i < indices_copy.size(); i++) {
            // If point i is lower than p0, then i becomes our new p0
            if (lower_point(points.col(indices_copy[i]), points.col(indices_copy[p0]))) {
                p0 = i;
            }
        }

        // Swap p0 with the first point
        std::swap(indices_copy[0], indices_copy[p0]);

        // Sort the points by polar angle with the first point in indices_copy
        sort_polar(indices_copy, points);

        // Initialize the stack
        std::vector<int> stack;
        stack.push_back(indices_copy[0]);
        stack.push_back(indices_copy[1]);
        stack.push_back(indices_copy[2]);

        // Process the remaining points
        for (size_t i = 3; i < indices_copy.size(); i++) {
            // Keep removing top while the angle formed by points next-to-top, top, and points[i] makes a clockwise turn
            while (
                stack.size() > 1
                && direction(points.col(stack[stack.size() - 2]), points.col(stack.back()), points.col(indices_copy[i]))
                       != Direction::COUNTERCLOCKWISE) {
                stack.pop_back();
            }
            stack.push_back(indices_copy[i]);
        }

        // The points in the stack are the vertices of the convex hull
        return stack;
    }

    /// @brief Partition the indices into `m` subsets of equal size
    /// @param indices Indices of the points to be partitioned, corresponding to indices in a matrix of points
    /// @param m The number of subsets to partition the indices into
    /// @return All the subsets, stored as a vector containing each subset
    std::vector<std::vector<int>> partition_points(const std::vector<int>& indices, int m) {
        std::vector<std::vector<int>> subsets(m);
        int n           = indices.size();
        int subset_size = n / m;
        int remainder   = n % m;

        for (int i = 0; i < m; i++) {
            int start  = i * subset_size + std::min(i, remainder);
            int end    = start + subset_size + (i < remainder ? 1 : 0);
            subsets[i] = std::vector<int>(indices.begin() + start, indices.begin() + end);
        }

        return subsets;
    }

    /// @brief From all the convex hulls, find the point that is most to the left or right, based on the comparison op
    /// @param hulls A vector of sub-hulls, each sub-hull is a vector of indices corresponding to points in `points`
    /// @param points All the points in our space, including points not to be used in the convex hull algorithm
    /// @param comp The comparison operator to use, either std::less or std::greater
    /// @return The index of the extreme point from all of the convex hulls
    int find_extreme_point(const std::vector<std::vector<int>>& hulls,
                           const Eigen::Matrix<double, 2, Eigen::Dynamic>& points,
                           const std::function<bool(double, double)>& comp) {
        // Initialise with the first point to start with
        int extreme = hulls[0][0];

        // Loop through all points, updating the leftmost if a new leftmost is found
        for (const auto& hull : hulls) {
            for (int index : hull) {
                if (comp(points(0, index), points(0, extreme))) {
                    extreme = index;
                }
            }
        }

        return extreme;
    }

    /// @brief Finds the upper tangent of the convex hulls
    /// Works by starting at the rightmost point and going counter-clockwise until the leftmost point is reached
    /// @param hulls A vector of sub-hulls, each sub-hull is a vector of indices corresponding to points in `points`
    /// @param points All the points in our space, including points not to be used in the convex hull algorithm
    /// @param leftmost The index of the leftmost point in all the hulls
    /// @param rightmost The index of the rightmost point in all the hulls
    /// @return The indices of the points that make up the upper tangent
    std::vector<int> find_upper_tangent(const std::vector<std::vector<int>>& hulls,
                                        const Eigen::Matrix<double, 2, Eigen::Dynamic>& points,
                                        int leftmost,
                                        int rightmost) {
        std::vector<int> upper_tangent{};

        // Start from the rightmost point and go counter-clockwise
        int current_point = rightmost;

        do {
            upper_tangent.push_back(current_point);

            // Find the next point that makes a counter-clockwise turn
            int next_point = hulls[0][0];
            for (const auto& hull : hulls) {
                for (int index : hull) {
                    double orientation =
                        (points(1, index) - points(1, current_point)) * (points(0, next_point) - points(0, index))
                        - (points(0, index) - points(0, current_point)) * (points(1, next_point) - points(1, index));

                    if (orientation < 0
                        || (orientation == 0
                            && (points.col(index) - points.col(current_point)).squaredNorm()
                                   > (points.col(next_point) - points.col(current_point)).squaredNorm())) {
                        next_point = index;
                    }
                }
            }

            current_point = next_point;
        } while (current_point != leftmost);

        return upper_tangent;
    }

    /// @brief Finds the lower tangent of the convex hulls
    /// Works by starting at the leftmost point and going counter-clockwise until the rightmost point is reached
    /// @param hulls A vector of sub-hulls, each sub-hull is a vector of indices corresponding to points in `points`
    /// @param points All the points in our space, including points not to be used in the convex hull algorithm
    /// @param leftmost The index of the leftmost point in all the hulls
    /// @param rightmost The index of the rightmost point in all the hulls
    /// @return The indices of the points that make up the upper tangent
    std::vector<int> find_lower_tangent(const std::vector<std::vector<int>>& hulls,
                                        const Eigen::Matrix<double, 2, Eigen::Dynamic>& points,
                                        int leftmost,
                                        int rightmost) {
        std::vector<int> lower_tangent{};

        // Start from the leftmost point and go clockwise
        int current_point = leftmost;

        do {
            lower_tangent.push_back(current_point);

            // Find the next point that makes a clockwise turn
            int next_point = hulls[0][0];
            for (const auto& hull : hulls) {
                for (int index : hull) {
                    double orientation =
                        (points(1, index) - points(1, current_point)) * (points(0, next_point) - points(0, index))
                        - (points(0, index) - points(0, current_point)) * (points(1, next_point) - points(1, index));

                    if (orientation < 0
                        || (orientation == 0
                            && (points.col(index) - points.col(current_point)).squaredNorm()
                                   > (points.col(next_point) - points.col(current_point)).squaredNorm())) {
                        next_point = index;
                    }
                }
            }

            current_point = next_point;
        } while (current_point != rightmost);

        return lower_tangent;
    }

    /// @brief Merges the convex hulls together to form a single convex hull
    /// @param hulls The convex hulls to be merged together
    /// @param points All the points in our space, including points not to be used in the convex hull algorithm
    /// @return The indices corresponding to points in `points` that make up the convex hull
    std::vector<int> merge_hulls(const std::vector<std::vector<int>>& hulls,
                                 const Eigen::Matrix<double, 2, Eigen::Dynamic>& points) {
        // Vector to hold the convex hull from merging the sub-hulls
        std::vector<int> merged_hull{};

        // Find the leftmost and rightmost points of all the hulls
        int leftmost  = find_extreme_point(hulls, points, std::less<double>());
        int rightmost = find_extreme_point(hulls, points, std::greater<double>());

        // Add the leftmost and rightmost points to the merged hull
        merged_hull.push_back(leftmost);

        // Find the upper and lower tangents of the hulls and add the points on the tangents to the merged hull
        std::vector<int> upper_tangent = find_upper_tangent(hulls, points, leftmost, rightmost);
        std::vector<int> lower_tangent = find_lower_tangent(hulls, points, leftmost, rightmost);

        // Add the points on the upper tangent to the merged hull in counter-clockwise order
        for (auto it = upper_tangent.rbegin(); it != upper_tangent.rend(); ++it) {
            merged_hull.push_back(*it);
        }

        // Add the points on the lower tangent to the merged hull in counter-clockwise order
        for (auto it = lower_tangent.rbegin(); it != lower_tangent.rend(); ++it) {
            merged_hull.push_back(*it);
        }

        return merged_hull;
    }

    /// @brief Algorithm to find a convex hull from a set of points using Chan's algorithm
    /// Chan's algorithm finds the convex hull of subsets of points and then merges those convex hulls together
    /// This is a type of 'divide-and-conquer' algorithm
    /// @param indices The indices corresponding to points in `points` that we want to use in the algorithm
    /// @param points All the points in our space, including points not to be used in the convex hull algorithm
    /// @return The indices corresponding to points in `points` that make up the convex hull
    inline std::vector<int> chans_convex_hull(const std::vector<int>& indices,
                                              const Eigen::Matrix<double, 2, Eigen::Dynamic>& points) {
        // Iterator variable to increase the number of subsets in each loop
        // Start it higher than 0 since we have a lot of points
        int t = 0;
        // The number of subsets to divide the points into, aims to find the smallest `m` such that the convex hull
        // algorithm returns a convex hull with no more than `m` points
        size_t m = 0;
        // Stores the best convex hull from these points, and is returned when the solution is found
        std::vector<int> final_hull{};

        // This must run at least once, to find a convex hull
        do {
            // Find the number of subsets
            t++;
            m = std::pow(2, t);
            // Create the subsets
            std::vector<std::vector<int>> subsets = partition_points(indices, m);
            // Find the convex hulls of each subset using graham scan
            std::vector<std::vector<int>> hulls{};
            for (auto& subset : subsets) {
                // We can't use this if a subset is too small
                if (subset.size() < 3) {
                    return final_hull;
                }
                hulls.push_back(graham_scan(subset, points));
            }

            // Merge the convex hulls together
            final_hull = merge_hulls(hulls, points);
        } while (final_hull.size() > m);  // keep trying until the smallest convex hull is found

        // Return the best hull
        return final_hull;
    }

    // Overloaded so we can directly call with our 3d matrix from the visual mesh
    inline std::vector<int> chans_convex_hull(const std::vector<int>& indices,
                                              const Eigen::Matrix<double, 3, Eigen::Dynamic>& points) {
        // Make the matrix of x-y points from the 3D points
        Eigen::Matrix<double, 2, Eigen::Dynamic> points_2d(2, points.cols());
        for (int i = 0; i < points.cols(); i++) {
            points_2d.col(i) << points(0, i), points(1, i);
        }

        return chans_convex_hull(indices, points_2d);
    }

    /// @brief Projects a three-dimensional vector onto the x-y plane
    /// @param v A three-dimensional vector
    /// @return The two-dimensional projection of v
    inline Eigen::Vector2d project_vector(const Eigen::Vector3d& v) {
        return Eigen::Vector2d(v.x() / v.z(), v.y() / v.z());
    }

    /// @brief Checks if a point is under the convex hull
    /// @tparam Iterator Iterator type for the rays
    /// @param point The point to check if it is under the convex hull
    /// @param rays_begin The beginning of the convex hull points
    /// @param rays_end The end of the convex hull points
    /// @param allow_boundary If true, then the point is considered under the hull if it is on the boundary
    /// @return True if the point is under the convex hull, false otherwise
    template <typename Iterator>
    inline bool point_under_hull(const Eigen::Vector3d& point,
                                 const Iterator rays_begin,
                                 const Iterator rays_end,
                                 const bool& allow_boundary = false) {
        const double theta = std::atan2(point.y(), point.x());
        auto lower_it = std::lower_bound(rays_begin, rays_end, theta, [&](const Eigen::Vector3d& p0, const double& b) {
            return std::atan2(p0.y(), p0.x()) < b;
        });
        auto upper_it = std::upper_bound(rays_begin, rays_end, theta, [&](const double& b, const Eigen::Vector3d& p0) {
            return b < std::atan2(p0.y(), p0.x());
        });

        if ((lower_it == rays_end) || (upper_it == rays_end)) {
            return false;
        }
        // lower_bound returns the first ray that has a theta value that is >= our point_theta value
        // taking the ray immediately before the lower_bound should give us a ray with theta < point_theta
        // upper_bound returns the first ray that has a theta value that is > our point_theta value
        const Eigen::Vector2d p0 = project_vector(lower_it == rays_begin ? *lower_it : *std::prev(lower_it));
        const Eigen::Vector2d p1 = project_vector(point);
        const Eigen::Vector2d p2 = project_vector(*upper_it);

        // Point should make a clockwise turn if it is under the convex hull.
        // It should be colinear if it is on the convex hull
        // If we allow the boundary then we only care that it isn't counter-clockwise
        // If we don't allow the boundary then we only care that it is clockwise
        return (allow_boundary ? direction(p0, p1, p2) != Direction::COUNTERCLOCKWISE
                               : direction(p0, p1, p2) == Direction::CLOCKWISE);  // Check for colinearity
    }

    /// @brief Sort the indices by theta
    /// @tparam Iterator Iterator type for the indices
    /// @param first The first index to sort
    /// @param last The last index to sort
    /// @param rays The rays that the indices correspond to
    /// @param world_offset The offset to apply to the theta values
    template <typename Iterator>
    void sort_by_theta(Iterator first,
                       Iterator last,
                       const Eigen::Matrix<double, 3, Eigen::Dynamic>& rays,
                       const double& world_offset) {
        std::sort(first, last, [&](const int& a, const int& b) {
            const Eigen::Vector3d& p0(rays.col(a));
            const Eigen::Vector3d& p1(rays.col(b));

            double theta0 = std::fmod(std::atan2(p0.y(), p0.x()) + M_PI - world_offset, 2.0 * M_PI);
            double theta1 = std::fmod(std::atan2(p1.y(), p1.x()) + M_PI - world_offset, 2.0 * M_PI);
            return theta0 < theta1;
        });
    }
}  // namespace utility::math::geometry
#endif  // UTILITY_MATH_GEOMETRY_CONVEXHULL_HPP
