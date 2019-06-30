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

#ifndef UTILITY_MATH_VISION_VISUALMESH_VISUALMESH_H
#define UTILITY_MATH_VISION_VISUALMESH_VISUALMESH_H

#include <Eigen/Core>
#include <iterator>
#include <queue>
#include <vector>

#include "utility/math/geometry/ConvexHull.h"

namespace utility {
namespace vision {
    namespace visualmesh {
        template <typename Iterator, typename Func>
        Iterator partition_points(Iterator first,
                                  Iterator last,
                                  const Eigen::MatrixXi& neighbours,
                                  Func&& pred,
                                  const std::initializer_list<int>& search_space = {0, 1, 2, 3, 4, 5}) {
            using value_type = typename std::iterator_traits<Iterator>::value_type;
            return std::partition(first, last, [&](const value_type& idx) {
                return pred(idx) && std::any_of(search_space.begin(), search_space.end(), [&](const auto& n) {
                           return !pred(neighbours(n, idx));
                       });
            });
        }

        template <typename Iterator>
        void cluster_points(Iterator first,
                            Iterator last,
                            const Eigen::MatrixXi& neighbours,
                            int min_cluster_size,
                            std::vector<std::vector<int>>& clusters) {

            // TODOs
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
                if (cluster.size() >= min_cluster_size) {
                    clusters.emplace_back(std::move(cluster));
                }
            }
        }

        template <typename Iterator, typename HorizonIt>
        Iterator check_green_horizon_side(Iterator first,
                                          Iterator last,
                                          HorizonIt horizon_first,
                                          HorizonIt horizon_last,
                                          const Eigen::Matrix<float, Eigen::Dynamic, 3>& rays,
                                          const bool& up   = true,
                                          const bool& down = true) {

            using value_type = typename std::iterator_traits<Iterator>::value_type;

            auto success = [&](const bool& a, const bool& b) {
                return (up && a && !down)     // We were looking for above and found them, we weren't looking for below
                       || (down && b && !up)  // We were looking for below and found them, we weren't looking for above
                       || (up && a && down && b);  // We were looking for everything and we found it
            };

            // Move any clusters that dont intersect the green horizon to the end of the list
            // We need to find one point above the green horizon and one below it
            return std::partition(first, last, [&](const value_type& cluster) {
                bool above = false, below = false;
                for (int idx = 0; idx < cluster.size() && !success(above, below); ++idx) {
                    if (utility::math::geometry::point_under_hull(
                            rays.row(cluster[idx]), horizon_first, horizon_last)) {
                        above = true;
                    }
                    else {
                        below = true;
                    }
                }
                return success(above, below);
            });
        }
    }  // namespace visualmesh
}  // namespace vision
}  // namespace utility

#endif  // UTILITY_MATH_VISION_VISUALMESH_VISUALMESH_H
