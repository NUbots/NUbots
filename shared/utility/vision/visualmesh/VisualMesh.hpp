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

    template <typename Iterator>
    void cluster_points(Iterator first,
                        Iterator last,
                        const Eigen::MatrixXi& neighbours,
                        int min_cluster_size,
                        std::vector<std::vector<int>>& clusters) {

        // TODO(VisionTeam):
        // 1) Do a reverse lookup-style reduction
        // 2) Convert to multi-partition and return a list of Iterators to each cluster

        using value_type = typename std::iterator_traits<Iterator>::value_type;

        // Do a DFS over all valid points and their neighbours to make connected clusters
        std::vector<bool> visited(std::distance(first, last), false);
        for (Iterator it = first; it != last; it = std::next(it)) {
            std::vector<Iterator> q;
            std::vector<value_type> cluster;

            // First element is always in the cluster
            q.push_back(it);

            while (!q.empty()) {
                // Get the next element to check
                Iterator current = q.back();
                q.pop_back();

                // Make sure we haven't seen this point before
                if (!visited[std::distance(first, current)]) {
                    // Add new point to cluster and mark it as seen
                    cluster.push_back(*current);
                    visited[std::distance(first, current)] = true;

                    // Find the current points neighbours
                    for (int n = 0; n < 6; ++n) {
                        const value_type neighbour_idx = neighbours(n, *current);
                        Iterator neighbour             = std::find(first, last, neighbour_idx);
                        if ((neighbour != last) && (!visited[std::distance(first, neighbour)])) {
                            q.push_back(neighbour);
                        }
                    }
                }
            }
            // Only add cluster to list if it meets minimum size requirment
            if (int(cluster.size()) >= min_cluster_size) {
                clusters.emplace_back(std::move(cluster));
            }
        }
    }

    template <typename Iterator, typename HorizonIt>
    Iterator check_green_horizon_side(Iterator first,
                                      Iterator last,
                                      HorizonIt horizon_first,
                                      HorizonIt horizon_last,
                                      const Eigen::Matrix<float, 3, Eigen::Dynamic>& rays,
                                      const bool& up   = true,
                                      const bool& down = true) {

        using value_type = typename std::iterator_traits<Iterator>::value_type;

        auto success = [&](const bool& a, const bool& b) {
            return (up && a && !down)     // We were looking for above and found them, we weren't looking for below
                   || (down && b && !up)  // We were looking for below and found them, we weren't looking for above
                   || ((up && a) || (down && b));  // We were looking for everything and we found it
        };

        // Move any clusters that dont intersect the green horizon to the end of the list
        // We need to find one point above the green horizon and one below it
        return std::partition(first, last, [&](const value_type& cluster) {
            bool above = false;
            bool below = false;
            for (unsigned int idx = 0; idx < cluster.size() && !success(above, below); ++idx) {
                if (utility::math::geometry::point_under_hull(rays.col(cluster[idx]), horizon_first, horizon_last)) {
                    above = true;
                }
                else {
                    below = true;
                }
            }
            return success(above, below);
        });
    }
}  // namespace utility::vision::visualmesh

#endif  // UTILITY_MATH_VISION_VISUALMESH_VISUALMESH_HPP
