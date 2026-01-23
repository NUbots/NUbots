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
        std::vector<bool> is_in_input(neighbours.cols(), false);
        for (int idx : values) {
            if (idx >= 0 && idx < int(is_in_input.size())) {
                is_in_input[idx] = true;
            }
        }

        // Track visited status of all points
        std::vector<bool> visited(neighbours.cols(), false);

        // Iterate through each index and form clusters via DFS
        for (int i = 0; i < N; ++i) {
            int seed = values[i];
            if (visited[seed]) {
                continue;
            }

            std::vector<int> cluster;
            std::vector<int> stack;
            stack.push_back(seed);

            // Perform DFS to find all connected points in the cluster
            while (!stack.empty()) {
                int current = stack.back();
                stack.pop_back();

                // If already visited, skip
                if (visited[current]) {
                    continue;
                }
                visited[current] = true;
                cluster.push_back(current);

                // Push unvisited neighbours that are also in the input set
                for (int n = 0; n < neighbours.rows(); ++n) {
                    int neigh = neighbours(n, current);
                    if (neigh >= 0 && neigh < int(visited.size()) && !visited[neigh] && is_in_input[neigh]) {
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

    auto check_green_horizon_side(std::vector<std::vector<int>>& clusters,
                                  const std::vector<Eigen::Vector3d>& horizon,
                                  const Eigen::Matrix<double, 3, Eigen::Dynamic>& rays,
                                  const bool& outside   = true,    // accept clusters completely outside
                                  const bool& inside    = true,    // accept clusters completely inside
                                  const bool& intersect = true) {  // accept clusters that go over the boundary

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

        // Move any clusters that don't intersect the green horizon to the end of the list
        // We need to find one point above the green horizon and one below it
        return std::partition(clusters.begin(), clusters.end(), [&](const std::vector<int>& cluster) {
            bool out = false;
            bool in  = false;

            for (unsigned int idx = 0; idx < cluster.size(); ++idx) {
                bool position =
                    utility::math::geometry::point_in_convex_hull(horizon, Eigen::Vector3d(rays.col(cluster[idx])));
                // Only set if it is in or out as we want to find across the cluster
                in  = position ? true : in;
                out = !position ? true : out;
            }
            return success(out, in);
        });
    }
}  // namespace utility::vision::visualmesh

#endif  // UTILITY_MATH_VISION_VISUALMESH_VISUALMESH_HPP
