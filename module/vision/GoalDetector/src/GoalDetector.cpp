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

#include "GoalDetector.h"

#include <cmath>

#include "RansacGoalModel.h"

#include "extension/Configuration.h"

#include "message/support/FieldDescription.h"
#include "message/vision/Goal.h"
#include "message/vision/GreenHorizon.h"

#include "utility/math/coordinates.h"
#include "utility/math/vision.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/vision/Vision.h"
#include "utility/vision/visualmesh/VisualMesh.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::support::FieldDescription;
    using message::vision::Goal;
    using message::vision::Goals;
    using message::vision::GreenHorizon;

    using utility::math::coordinates::cartesianToSpherical;

    static constexpr int GOAL_INDEX = 0;

    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Trigger the same function when either update
        on<Configuration>("GoalDetector.yaml").then([this](const Configuration& cfg) {
            config.confidence_threshold = cfg["confidence_threshold"].as<float>();
            config.cluster_points       = cfg["cluster_points"].as<int>();
            config.disagreement_ratio   = cfg["disagreement_ratio"].as<float>();
            config.goal_angular_cov =
                convert<double, 3>(cfg["goal_angular_cov"].as<arma::vec>()).cast<float>().asDiagonal();
            config.debug = cfg["debug"].as<bool>();
        });

        on<Trigger<GreenHorizon>, With<FieldDescription>>().then(
            "Goal Detector", [this](const GreenHorizon& horizon, const FieldDescription& field) {
                // Convenience variables
                const auto& cls                                     = horizon.mesh->classifications;
                const auto& neighbours                              = horizon.mesh->neighbourhood;
                const Eigen::Matrix<float, Eigen::Dynamic, 3>& rays = horizon.mesh->rays;
                const float world_offset                            = std::atan2(horizon.Hcw(0, 1), horizon.Hcw(0, 0));

                // Get some indices to partition
                std::vector<int> indices(horizon.mesh->indices.size());
                std::iota(indices.begin(), indices.end(), 0);

                // Partition the indices such that we only have the goal points that dont have goal surrounding them
                auto boundary = utility::vision::visualmesh::partition_points(
                    indices.begin(), indices.end(), neighbours, [&](const int& idx) {
                        return idx == indices.size() || (cls(idx, GOAL_INDEX) >= config.confidence_threshold);
                    });

                // Discard indices that are not on the boundary and are not below the green horizon
                indices.resize(std::distance(indices.begin(), boundary));

                // Cluster all points into ball candidates
                // Points are clustered based on their connectivity to other ball points
                // Clustering is down in two steps
                // 1) We take the set of goal points found above and partition them into potential clusters by
                //    a) Add the first point and its goal neighbours to a cluster
                //    b) Find all other goal points who are neighbours of the points in the cluster
                //    c) Partition all of the indices that are in the cluster
                //    d) Repeat a-c for all points that were not partitioned
                //    e) Delete all partitions smaller than a given threshold
                // 2) Discard all clusters that do not intersect the green horizon
                std::vector<std::vector<int>> clusters;
                utility::vision::visualmesh::cluster_points(
                    indices.begin(), indices.end(), neighbours, config.cluster_points, clusters);

                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));
                }

                auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(
                    clusters.begin(), clusters.end(), horizon.horizon.begin(), horizon.horizon.end(), rays, true, true);
                clusters.resize(std::distance(clusters.begin(), green_boundary));

                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Found {} clusters below green horizon", clusters.size()));
                }

                if (clusters.size() > 0) {
                    auto goals = std::make_unique<Goals>();
                    goals->goals.reserve(clusters.size());

                    goals->camera_id = horizon.camera_id;
                    goals->timestamp = horizon.timestamp;
                    goals->Hcw       = horizon.Hcw;

                    for (auto& cluster : clusters) {
                        Goal g;
                        // Split cluster into left-side and right-side
                        auto right = std::partition(cluster.begin(), cluster.end(), [&](const int& idx) {
                            // Return true if the left neighbour is NOT a goal point
                            const int neighbour_idx = neighbours(idx, 2);
                            if (neighbour_idx == indices.size()) {
                                return true;
                            }
                            if (cls(neighbour_idx, GOAL_INDEX) < config.confidence_threshold) {
                                return true;
                            }
                            return false;
                        });
                        auto other = std::partition(right, cluster.end(), [&](const int& idx) {
                            // Return true if the right neighbour is NOT a goal point
                            const int neighbour_idx = neighbours(idx, 3);
                            if (neighbour_idx == indices.size()) {
                                return true;
                            }
                            if (cls(neighbour_idx, GOAL_INDEX) < config.confidence_threshold) {
                                return true;
                            }
                            return false;
                        });

                        if (config.debug) {
                            log<NUClear::DEBUG>(
                                fmt::format("Cluster split into {} left points, {} right points, {} top/bottom points",
                                            std::distance(cluster.begin(), right),
                                            std::distance(right, other),
                                            std::distance(other, cluster.end())));
                        }

                        // Calculate average of left_xy and right_xy and find lowest z point
                        Eigen::Vector3f left_side    = Eigen::Vector3f::Zero();
                        Eigen::Vector3f right_side   = Eigen::Vector3f::Zero();
                        Eigen::Vector3f bottom_point = Eigen::Vector3f::Zero();
                        for (auto it = cluster.begin(); it != right; it = std::next(it)) {
                            const Eigen::Vector3f& p0(rays.row(*it));
                            left_side += p0;
                            if (p0.z() < bottom_point.z()) {
                                bottom_point = p0;
                            }
                        }
                        for (auto it = right; it != other; it = std::next(it)) {
                            const Eigen::Vector3f& p0(rays.row(*it));
                            right_side += p0;
                            if (p0.z() < bottom_point.z()) {
                                bottom_point = p0;
                            }
                        }
                        for (auto it = other; it != cluster.end(); it = std::next(it)) {
                            const Eigen::Vector3f& p0(rays.row(*it));
                            if (p0.z() < bottom_point.z()) {
                                bottom_point = p0;
                            }
                        }
                        left_side /= std::distance(cluster.begin(), right);
                        right_side /= std::distance(right, other);

                        // Calculate bottom middle of goal post, z is calculated above
                        bottom_point.x() = (left_side.x() + right_side.x()) * 0.5f;
                        bottom_point.y() = (left_side.y() + right_side.y()) * 0.5f;
                        bottom_point     = horizon.Hcw.topLeftCorner<3, 3>().cast<float>() * bottom_point;

                        // https://en.wikipedia.org/wiki/Angular_diameter
                        Eigen::Vector3f middle = ((left_side + right_side) * 0.5f).normalized();
                        float radius           = middle.dot(left_side.normalized());
                        float distance =
                            field.dimensions.goalpost_width * radius * 0.5f / std::sqrt(1.0f - radius * radius);

                        // Normal should be orthogonal to both the viewing direction (bottom_point) and world z
                        g.center_line.normal   = bottom_point.cross(horizon.Hcw.topLeftCorner<3, 3>().cast<float>()
                                                                  * Eigen::Vector3f::UnitZ());
                        g.center_line.distance = distance;

                        // Attach the measurement to the object (distance from camera to bottom center of post)
                        g.measurements.push_back(Goal::Measurement());
                        g.measurements.back().type     = Goal::MeasurementType::CENTRE;
                        g.measurements.back().position = cartesianToSpherical(Eigen::Vector3f(bottom_point * distance));
                        g.measurements.back().covariance =
                            config.goal_angular_cov * Eigen::Vector3f(distance, 1, 1).asDiagonal();

                        // Angular positions from the camera
                        g.screen_angular = cartesianToSpherical(bottom_point).tail<2>();
                        g.angular_size   = Eigen::Vector2f::Constant(std::acos(radius));
                    }

                    // We can see two goal posts, check to make sure they belong to the same goals and then assign left
                    // and right sides
                    if (goals->goals.size() == 2) {
                        // Divide by distance to get back to unit vectors
                        const Eigen::Vector3f rGCc0 =
                            goals->goals[0].measurements.back().position / goals->goals[0].center_line.distance;
                        const Eigen::Vector3f rGCc1 =
                            goals->goals[1].measurements.back().position / goals->goals[1].center_line.distance;
                        const Eigen::Vector3f cam_space_z =
                            horizon.Hcw.topLeftCorner<3, 3>().cast<float>() * Eigen::Vector3f::UnitZ();

                        // Law of consines
                        const float a = goals->goals[0].center_line.distance * goals->goals[0].center_line.distance;
                        const float b = goals->goals[1].center_line.distance * goals->goals[1].center_line.distance;
                        const float cos_gamma    = rGCc0.dot(rGCc1);
                        const float goal_width   = a + b - 2 * a * b * cos_gamma;
                        const float actual_width = field.dimensions.goal_width * field.dimensions.goal_width;

                        // Check the width between the posts, if they are close enough then assign left and right sides
                        if (std::abs(goal_width - actual_width) / std::max(goal_width, actual_width)
                            < config.disagreement_ratio) {
                            // Direction (determined by RHR) needed to turn to get from one post to the other
                            const float direction = rGCc0.cross(rGCc1).dot(cam_space_z);

                            // Anti-clockwise turn from rGCc0 to rGCc1 around cam_space_z
                            // ==> rGCc1 is to the left of rGCc0
                            if (direction > 0.0f) {
                                goals->goals.at(0).side = Goal::Side::RIGHT;
                                goals->goals.at(1).side = Goal::Side::LEFT;
                            }
                            // Clockwise turn from rGCc0 to rGCc1 around cam_space_z
                            // ==> rGCc0 is to the left of rGCc1
                            else if (direction < 0.0f) {
                                goals->goals.at(0).side = Goal::Side::LEFT;
                                goals->goals.at(1).side = Goal::Side::RIGHT;
                            }

                            // If direction is 0 then rGCc0 is parallel to rGCc1
                            // This either means that the two goal posts are on top of each (rGCc0 == rGCc1), or
                            // they are anti-parallel and the two goal posts are at opposite ends of the field
                            //
                            // Because of clustering, it should not be possible that rGCc0 and rGCc1 are equal
                        }
                    }
                    // We can see three goal posts
                    // Two probably belong to one set of goals and the third to a different set
                    // Figure out which is which
                    else if (goals->goals.size() == 3) {
                    }
                    // We can see four goal posts
                    // We can probably see two sets of goal posts, figure out the pairs
                    else if (goals->goals.size() == 4) {
                    }

                    emit(std::move(goals));
                }
            });
    }
}  // namespace vision
}  // namespace module
