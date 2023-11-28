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
#include "GreenHorizonDetector.hpp"

#include <fmt/format.h>
#include <numeric>
#include <set>

#include "extension/Configuration.hpp"

#include "message/vision/GreenHorizon.hpp"
#include "message/vision/VisualMesh.hpp"

#include "utility/math/geometry/ConvexHull.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::vision {

    using extension::Configuration;
    using message::vision::GreenHorizon;
    using message::vision::VisualMesh;
    using utility::nusight::graph;

    GreenHorizonDetector::GreenHorizonDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("GreenHorizonDetector.yaml").then([this](const Configuration& config) {
            // Use configuration here from file GreenHorizonDetector.yaml
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.confidence_threshold = config["confidence_threshold"].as<double>();
            cfg.cluster_points       = config["cluster_points"].as<uint>();
        });

        on<Trigger<VisualMesh>, Buffer<2>>().then("Green Horizon", [this](const VisualMesh& mesh) {
            // Convenience variables
            const auto& cls                                      = mesh.classifications;
            const auto& neighbours                               = mesh.neighbourhood;
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& uPCw = mesh.rays.cast<double>();
            const Eigen::Isometry3d Hwc                          = mesh.Hcw.inverse();
            const uint32_t LINE_INDEX                            = mesh.class_map.at("line");
            const uint32_t FIELD_INDEX                           = mesh.class_map.at("field");

            // Convert rays to world space
            Eigen::Matrix<double, 3, Eigen::Dynamic> rPWw(3, uPCw.cols());

            // Loop over each position and add the transformed ray
            for (int idx = 0; idx < uPCw.cols(); idx++) {
                rPWw.col(idx) = uPCw.col(idx) * std::abs(Hwc.translation().z() / uPCw.col(idx).z()) + Hwc.translation();
            }

            // Get some indices to partition
            std::vector<int> indices(mesh.indices.size());
            std::iota(indices.begin(), indices.end(), 0);

            // Set up the check for if a point is on the field
            auto is_on_field = [&](const int& idx) {
                return cls(FIELD_INDEX, idx) + cls(LINE_INDEX, idx) >= cfg.confidence_threshold;
            };

            // Partition the indices such that we only have the field points
            auto field_points = std::partition(indices.begin(), indices.end(), is_on_field);
            indices.resize(std::distance(indices.begin(), field_points));

            // Cluster the points and find the closest to the robot, as this is most likely to be the field
            std::vector<std::vector<int>> clusters;
            utility::vision::visualmesh::cluster_points(indices.begin(),
                                                        indices.end(),
                                                        neighbours,
                                                        cfg.cluster_points,
                                                        clusters);

            log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));

            // Prevent issues if there are no clusters
            if (clusters.size() == 0) {
                log<NUClear::DEBUG>("No clusters found, cannot form a green horizon");
                return;
            }

            // Get points from the camera since we want to measure the distance from the robot
            Eigen::Matrix<double, 3, Eigen::Dynamic> rPCc(3, rPWw.cols());

            // Loop over each position and change the coordinate space
            for (int idx = 0; idx < rPWw.cols(); idx++) {
                rPCc.col(idx) = Hwc.inverse() * rPWw.col(idx);
            }

            // Get the closest distance to the robot from all points in the cluster
            auto get_closest_distance = [&](const std::vector<int>& cluster) {
                int closest_index = *std::min_element(cluster.begin(), cluster.end(), [&](int a, int b) {
                    return rPCc.col(a).norm() < rPCc.col(b).norm();
                });
                return rPCc.col(closest_index).norm();
            };

            // Find the cluster closest to the robot
            auto closest_cluster_it = std::min_element(clusters.begin(),
                                                       clusters.end(),
                                                       [&](const std::vector<int>& a, const std::vector<int>& b) {
                                                           return get_closest_distance(a) < get_closest_distance(b);
                                                       });

            for (const auto& cluster : clusters) {
                log<NUClear::DEBUG>(fmt::format("Cluster with {} points and distance {}",
                                                cluster.size(),
                                                get_closest_distance(cluster)));
            }

            log<NUClear::DEBUG>(fmt::format("Closest cluster has {} points", closest_cluster_it->size()));

            // The closest cluster to the robot is the field cluster
            auto field_cluster = *closest_cluster_it;

            // Partition the cluster such that we only have the boundary points of the cluster
            auto boundary = utility::vision::visualmesh::boundary_points(field_cluster.begin(),
                                                                         field_cluster.end(),
                                                                         neighbours,
                                                                         is_on_field);

            // Discard points from the cluster that are not on the boundary
            field_cluster.resize(std::distance(field_cluster.begin(), boundary));

            // Graph the field cluster points if debugging
            if (log_level <= NUClear::DEBUG) {
                for (int idx : field_cluster) {
                    emit(graph("Field cluster point", rPWw.col(idx).x(), rPWw.col(idx).y()));
                }
            }

            // The remaining points are on the boundary of the field
            // They may also appear around other objects such as the ball
            log<NUClear::DEBUG>(
                fmt::format("Found {} points on the boundary of the cluster to create a convex hull with",
                            field_cluster.size()));

            // Convex hull algorithms require at least three points
            if (field_cluster.size() < 3) {
                log<NUClear::DEBUG>("Not enough points to make a convex hull");
                return;
            }

            // Find the convex hull of the field points
            auto hull_indices = utility::math::geometry::chans_convex_hull(field_cluster, rPWw);

            // Graph the convex hull if debugging
            if (log_level <= NUClear::DEBUG) {
                for (int idx : hull_indices) {
                    emit(graph("Convex hull point", rPWw.col(idx).x(), rPWw.col(idx).y()));
                }
            }

            auto msg = std::make_unique<GreenHorizon>();

            // Preserve mesh so that anyone using the GreenHorizon can access the original data
            msg->mesh = const_cast<VisualMesh*>(&mesh)->shared_from_this();

            msg->id        = mesh.id;
            msg->Hcw       = mesh.Hcw;
            msg->timestamp = mesh.timestamp;
            msg->class_map = mesh.class_map;

            // If using ground truth, add it to the message
            if (mesh.vision_ground_truth.exists) {
                msg->vision_ground_truth = mesh.vision_ground_truth;
            }

            // Add the unit vectors of the convex hull to the green horizon message
            msg->horizon.reserve(hull_indices.size());
            for (const auto& idx : hull_indices) {
                msg->horizon.emplace_back(uPCw.col(idx));
            }

            log<NUClear::DEBUG>(fmt::format("Calculated a convex hull with {} points from a boundary with {} points",
                                            hull_indices.size(),
                                            field_cluster.size()));

            emit(std::move(msg));
        });
    }
}  // namespace module::vision
