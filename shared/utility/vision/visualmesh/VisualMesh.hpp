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

#ifndef UTILITY_MATH_VISION_VISUALMESH_VISUALMESH_HPP
#define UTILITY_MATH_VISION_VISUALMESH_VISUALMESH_HPP

#include <Eigen/Core>
#include <iterator>
#include <queue>
#include <vector>

#include "utility/math/geometry/ConvexHull.hpp"

namespace utility::vision::visualmesh {
    template <typename Iterator, typename Func>
    Iterator partition_points(Iterator first,
                              Iterator last,
                              const Eigen::MatrixXi& neighbours,
                              Func&& pred,  // function determining if the index has a high enough confidence to use
                              const std::initializer_list<int>& search_space = {0, 1, 2, 3, 4, 5}) {
        using value_type = typename std::iterator_traits<Iterator>::value_type;

        return std::partition(first, last, [&](const value_type& idx) {
            // Check if this index satisfies our confidence requirements
            bool prediction = pred(idx);
            // Check if any of the required neighbours satisfy our confidence requirements
            // Required neighbours are from the search space list
            bool check = std::any_of(search_space.begin(), search_space.end(), [&](const auto& n) {
                return !pred(neighbours(n, idx));
            });

            return prediction && check;
        });
    }

    template <typename Iterator, typename Func>
    Iterator boundary_points(Iterator first,
                             Iterator last,
                             const Eigen::MatrixXi& neighbours,
                             Func&& pred,  // function determining if the index has a high enough confidence to use
                             const std::initializer_list<int>& search_space = {0, 1, 2, 3, 4, 5}) {
        using value_type = typename std::iterator_traits<Iterator>::value_type;

        // Check if any of the required neighbours satisfy our confidence requirements
        // Required neighbours are from the search space list
        return std::partition(first, last, [&](const value_type& idx) {
            return std::any_of(search_space.begin(), search_space.end(), [&](const auto& n) {
                return !pred(neighbours(n, idx));
            });
        });
    }

    /**
     * @brief Clusters connected mesh point indices using DFS.
     *
     * @tparam Iterator An iterator over integers representing point indices [0, N).
     * @param first Iterator to the beginning of point indices.
     * @param last Iterator to the end of point indices.
     * @param neighbours MxN matrix where each column contains M neighbour indices for a point.
     * @param min_cluster_size Minimum number of points for a cluster to be valid.
     * @param clusters Output vector of clusters, each a vector of indices.
     */
    template <typename Iterator>
    void cluster_points(Iterator first,
                        Iterator last,
                        const Eigen::MatrixXi& neighbours,
                        int min_cluster_size,
                        std::vector<std::vector<int>>& clusters) {
        using value_type = typename std::iterator_traits<Iterator>::value_type;

        const int N = std::distance(first, last);
        // If there are no points, return immediately
        if (N == 0) {
            return;
        }

        // Copy values (point indices) for indexed access
        std::vector<value_type> values(first, last);

        // Lookup table: which indices are in the input set
        std::vector<char> is_in_input(neighbours.cols(), false);
        for (int idx : values) {
            if (idx >= 0 && idx < int(is_in_input.size())) {
                is_in_input[idx] = true;
            }
        }

        // Track visited status of all points
        std::vector<char> visited(neighbours.cols(), false);

        // Iterate through each index and form clusters via DFS
        for (int i = 0; i < N; ++i) {
            int seed = values[i];
            if (visited[seed]) {
                continue;
            }

            std::vector<int> cluster;
            std::vector<int> stack;
            stack.push_back(seed);
            visited[seed] = true;

            // Perform DFS to find all connected points in the cluster
            while (!stack.empty()) {
                int current = stack.back();
                stack.pop_back();
                cluster.push_back(current);

                // Push unvisited neighbours that are also in the input set
                for (int n = 0; n < neighbours.rows(); ++n) {
                    int neigh = neighbours(n, current);
                    if (neigh >= 0 && neigh < int(visited.size()) && !visited[neigh] && is_in_input[neigh]) {
                        visited[neigh] = true;
                        stack.push_back(neigh);
                    }
                }
            }

            // If the cluster is large enough, add it to the output
            if (int(cluster.size()) >= min_cluster_size) {
                clusters.emplace_back(std::move(cluster));
            }
        }
    }

    std::vector<char> get_green_horizon_side_mask(
        const std::vector<std::vector<int>>& clusters,
        const std::vector<Eigen::Vector3d>& horizon,
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& rays,
        const bool outside   = true,    // accept clusters completely outside
        const bool inside    = true,    // accept clusters completely inside
        const bool intersect = true) {  // accept clusters that go over the boundary

        auto success = [&](const bool cluster_outside, const bool cluster_inside) {
            // Cluster is completely outside
            if (cluster_outside && !cluster_inside) {
                return outside;
            }
            // Cluster is completely inside
            if (!cluster_outside && cluster_inside) {
                return inside;
            }
            // Cluster intersects the boundary
            if (cluster_outside && cluster_inside) {
                return intersect;
            }
            // Last option is it's neither in or out
            NUClear::log<NUClear::LogLevel::ERROR>(
                "Cluster is neither inside or outside the green horizon. This is bad.");
            return false;
        };

        std::vector<char> is_accepted;
        is_accepted.reserve(clusters.size());

        for (const auto& cluster : clusters) {
            bool out = false;
            bool in  = false;

            for (size_t idx{}; idx < cluster.size(); ++idx) {
                bool position =
                    utility::math::geometry::point_in_convex_hull(horizon, Eigen::Vector3d(rays.col(cluster[idx])));
                // Only set if it is in or out as we want to find across the cluster
                in  = position ? true : in;
                out = !position ? true : out;

                if (out && in) {
                    // known to be an intersection case
                    break;
                }
            }

            is_accepted.push_back(success(out, in));
        }

        return is_accepted;
    }

    auto check_green_horizon_side(std::vector<std::vector<int>>& clusters,
                                  const std::vector<Eigen::Vector3d>& horizon,
                                  const Eigen::Matrix<double, 3, Eigen::Dynamic>& rays,
                                  const bool outside   = true,    // accept clusters completely outside
                                  const bool inside    = true,    // accept clusters completely inside
                                  const bool intersect = true) {  // accept clusters that go over the boundary

        auto success = [&](const bool& cluster_outside, const bool& cluster_inside) {
            // Cluster is completely outside
            if (cluster_outside && !cluster_inside) {
                return outside;
            }
            // Cluster is completely inside
            if (!cluster_outside && cluster_inside) {
                return inside;
            }
            // Cluster intersects the boundary
            if (cluster_outside && cluster_inside) {
                return intersect;
            }
            // Last option is it's neither in or out
            NUClear::log<NUClear::LogLevel::ERROR>(
                "Cluster is neither inside or outside the green horizon. This is bad.");
            return false;
        };

        // Move any clusters that don't meet success criterion to the end of the list
        // We need to find if there are points above the green horizon and/or one below it
        return std::partition(clusters.begin(), clusters.end(), [&](const std::vector<int>& cluster) {
            bool out = false;
            bool in  = false;

            for (unsigned int idx = 0; idx < cluster.size(); ++idx) {
                bool position =
                    utility::math::geometry::point_in_convex_hull(horizon, Eigen::Vector3d(rays.col(cluster[idx])));
                // Only set if it is in or out as we want to find across the cluster
                in  = position ? true : in;
                out = !position ? true : out;

                if (out && in) {
                    // known to be an intersection case
                    break;
                }
            }
            return success(out, in);
        });
    }

    /**
     * @brief Count number of mesh points bounded in cone from camera using DFS.
     *
     * @param indice_collection Collection of unique indices that are known to be bounded in the cone
     * @param neighbours MxN matrix where each column contains M neighbour indices for a point.
     * @param uBCw The centre of the bounded cone represented as a unit vector from camera to ball in world space
     * @param radius Angular radius of the cone, equal to cos(theta)
     * @param uPCw Unit vectors from camera to a point in the mesh in world space
     * @return number of bounded points the mesh has in this cone
     */
    size_t find_number_bounded_points(const std::vector<std::vector<int>*>& indice_collection,
                                      const Eigen::MatrixXi& neighbours,
                                      const Eigen::Vector3d uBCw,
                                      const double radius,
                                      const Eigen::Matrix<double, 3, Eigen::Dynamic>& uPCw) {
        // track visited points
        std::vector<char> visited(neighbours.cols(), false);
        std::vector<int> stack;
        size_t num_visited{};

        // indices are known to be unique and bounded in the cone so they are added to the stack without checks
        for (const std::vector<int>* indices : indice_collection) {
            stack.insert(stack.end(), indices->begin(), indices->end());
        }

        // indices in the stack have been visited
        for (int index : stack) {
            visited[index] = true;
        }
        num_visited += stack.size();

        // depth first search where if a points neighbours are bounded by the cone they are connected
        while (!stack.empty()) {
            int current = stack.back();
            stack.pop_back();

            for (int i{}; i < neighbours.rows(); ++i) {
                int neigh = neighbours(i, current);
                // radius and uPCw.col(neigh).dot(uBCw) are in cos(theta), so being larger than radius bounds it inside
                // the cone. Neighbour >= 0 so can be safely type cast into size_t.
                if (neigh >= 0 && static_cast<size_t>(neigh) < visited.size() && !visited[neigh]
                    && uPCw.col(neigh).dot(uBCw) >= radius) {
                    stack.push_back(neigh);
                    visited[neigh] = true;
                    ++num_visited;
                }
            }
        }

        return num_visited;
    }

    /**
     * @brief Finds central axis of a cluster
     * Adds up all the unit vectors of each point (camera to point in world space) in the cluster to find an average
     * vector, which represents the central cone axis
     * @param cluster A collection of points
     * @param uPCw Unit vector from camera to a point in the mesh in world space
     * @return Eigen::Vector3d, uBCw: unit vector from camera to ball central axis in world space
     */
    Eigen::Vector3d find_cluster_central_axis(const std::vector<int>& cluster,
                                              const Eigen::Matrix<double, 3, Eigen::Dynamic>& uPCw) {
        Eigen::Vector3d uBCw = Eigen::Vector3d::Zero();
        for (const auto& idx : cluster) {
            uBCw += uPCw.col(idx);
        }
        uBCw.normalize();
        return uBCw;
    }

    /**
     * @brief Finds the angular radius of the cluster from the camera
     * Find the ray (uPCw) with the greatest distance from the central axis (uBCw) to then determine the
     * largest angular radius possible from the edge points available. Equal to cos(theta), where theta
     * is the angle between the central ball axis (uBCw) and the edge of the ball.
     * @param cluster A collection of points
     * @param uPCw Unit vector from camera to a point in the mesh in world space
     * @param uBCw Unit vector from camera to ball central axis in world space
     * @return double, angular radius of cluster equal to cos(theta)
     */
    double find_cluster_angular_radius(const std::vector<int>& cluster,
                                       const Eigen::Matrix<double, 3, Eigen::Dynamic>& uPCw,
                                       const Eigen::Vector3d& uBCw) {
        double radius{1.0};
        for (const auto& idx : cluster) {
            // Unit vector from the camera to the ball edge, in world space
            const Eigen::Vector3d& uECw(uPCw.col(idx));
            // Find the vector that gives the largest angle between the central axis and ball edge
            // Radius is cos(theta), where theta is the angle, so a smaller radius gives a larger angle.
            radius = uBCw.dot(uECw) < radius ? uBCw.dot(uECw) : radius;
        }

        return radius;
    }

}  // namespace utility::vision::visualmesh

#endif  // UTILITY_MATH_VISION_VISUALMESH_VISUALMESH_HPP
