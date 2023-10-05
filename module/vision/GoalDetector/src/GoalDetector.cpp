/*
 * This file is part of NUbots Codebase.
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

#include "GoalDetector.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <numeric>

#include "extension/Configuration.hpp"

#include "message/support/FieldDescription.hpp"
#include "message/vision/Goal.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/math/geometry/ConvexHull.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::support::FieldDescription;
    using message::vision::Goal;
    using message::vision::Goals;
    using message::vision::GreenHorizon;

    using utility::math::coordinates::cartesianToReciprocalSpherical;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::support::Expression;

    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Trigger the same function when either update
        on<Configuration>("GoalDetector.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            config.confidence_threshold       = cfg["confidence_threshold"].as<float>();
            config.cluster_points             = cfg["cluster_points"].as<int>();
            config.disagreement_ratio         = cfg["disagreement_ratio"].as<float>();
            config.goal_projection_covariance = Eigen::Vector3f(cfg["goal_projection_covariance"].as<Expression>());
            config.use_median                 = cfg["use_median"].as<bool>();
            config.max_goal_distance          = cfg["max_goal_distance"].as<float>();
        });

        on<Trigger<GreenHorizon>, With<FieldDescription>, Buffer<2>>().then(
            "Goal Detector",
            [this](const GreenHorizon& horizon, const FieldDescription& field) {
                // Convenience variables
                const auto& cls                                     = horizon.mesh->classifications;
                const auto& neighbours                              = horizon.mesh->neighbourhood;
                const Eigen::Matrix<float, 3, Eigen::Dynamic>& rays = horizon.mesh->rays;
                const float world_offset                            = std::atan2(horizon.Hcw(0, 1), horizon.Hcw(0, 0));
                const int GOAL_INDEX                                = horizon.class_map.at("goal");

                // Get some indices to partition
                std::vector<int> indices(horizon.mesh->indices.size());
                std::iota(indices.begin(), indices.end(), 0);

                // Partition the indices such that we only have the goal points that dont have goal surrounding them
                auto boundary = utility::vision::visualmesh::partition_points(
                    indices.begin(),
                    indices.end(),
                    neighbours,
                    [&](const int& idx) {
                        return idx == int(indices.size()) || (cls(GOAL_INDEX, idx) >= config.confidence_threshold);
                    });

                // Discard indices that are not on the boundary and are not below the green horizon
                indices.resize(std::distance(indices.begin(), boundary));

                // Cluster all points into goal candidates
                // Points are clustered based on their connectivity to other goal points
                // Clustering is down in two steps
                // 1) We take the set of goal points found above and partition them into potential clusters by
                //    a) Add the first point and its goal neighbours to a cluster
                //    b) Find all other goal points who are neighbours of the points in the cluster
                //    c) Partition all of the indices that are in the cluster
                //    d) Repeat a-c for all points that were not partitioned
                //    e) Delete all partitions smaller than a given threshold
                // 2) Discard all clusters that do not intersect the green horizon
                std::vector<std::vector<int>> clusters;
                utility::vision::visualmesh::cluster_points(indices.begin(),
                                                            indices.end(),
                                                            neighbours,
                                                            config.cluster_points,
                                                            clusters);

                log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));

                auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(clusters.begin(),
                                                                                            clusters.end(),
                                                                                            horizon.horizon.begin(),
                                                                                            horizon.horizon.end(),
                                                                                            rays,
                                                                                            true,
                                                                                            true);
                clusters.resize(std::distance(clusters.begin(), green_boundary));

                log<NUClear::DEBUG>(fmt::format("Found {} clusters that intersect the green horizon", clusters.size()));

                // Find overlapping clusters and merge them
                for (auto it = clusters.begin(); it != clusters.end(); it = std::next(it)) {
                    // Get the largest and smallest theta values
                    auto range_a = std::minmax_element(it->begin(), it->end(), [&rays](const int& a, const int& b) {
                        return std::atan2(rays(1, a), rays(0, a)) < std::atan2(rays(1, b), rays(0, b));
                    });

                    const float min_a = std::atan2(rays(1, *range_a.first), rays(0, *range_a.first));
                    const float max_a = std::atan2(rays(1, *range_a.second), rays(0, *range_a.second));

                    for (auto it2 = std::next(it); it2 != clusters.end();) {
                        // Get the largest and smallest theta values
                        auto range_b =
                            std::minmax_element(it2->begin(), it2->end(), [&rays](const int& a, const int& b) {
                                return std::atan2(rays(1, a), rays(0, a)) < std::atan2(rays(1, b), rays(0, b));
                            });

                        const float min_b = std::atan2(rays(1, *range_b.first), rays(0, *range_b.first));
                        const float max_b = std::atan2(rays(1, *range_b.second), rays(0, *range_b.second));

                        // The clusters are overlapping, merge them
                        if (((min_a <= min_b) && (min_b <= max_a)) || ((min_b <= min_a) && (min_a <= max_b))) {
                            // Append the second cluster on to the first
                            it->insert(it->end(), it2->begin(), it2->end());
                            // Delete the second cluster
                            it2 = clusters.erase(it2);
                        }
                        else {
                            it2 = std::next(it2);
                        }
                    }
                }

                log<NUClear::DEBUG>(fmt::format("{} clusters remaining after merging overlaps", clusters.size()));

                auto goals = std::make_unique<Goals>();
                if (!clusters.empty()) {
                    goals->goals.reserve(clusters.size());

                    goals->id        = horizon.id;
                    goals->timestamp = horizon.timestamp;
                    goals->Hcw       = horizon.Hcw;

                    for (auto& cluster : clusters) {
                        // Find the point in the cluster that is closest to the camera and use that as the goal
                        auto closest_uPCw_idx =
                            std::min_element(cluster.begin(), cluster.end(), [&](const int& a, const int& b) {
                                return rays.col(a).z() < rays.col(b).z();
                            });
                        Eigen::Vector3d closest_uPCw = Eigen::Vector3d(rays.col(*closest_uPCw_idx).cast<double>());
                        goals->points.push_back(closest_uPCw);
                        // Project goal point onto the field plane
                        auto Hwc = horizon.Hcw.inverse();
                        Eigen::Vector3d rPWw =
                            closest_uPCw * std::abs(Hwc.translation().z() / closest_uPCw.z()) + Hwc.translation();
                        goals->rPWw.push_back(rPWw);
                        log<NUClear::DEBUG>("Goal at: ", rPWw.transpose());
                    }
                }
                log<NUClear::DEBUG>("Found ", goals->points.size(), " goals");
                emit(goals);
            });
    }
}  // namespace module::vision
