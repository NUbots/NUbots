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

#include "utility/math/geometry/ChanConvexHull.hpp"
#include "utility/math/geometry/ConvexHull.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::vision {

    using extension::Configuration;
    using message::vision::VisualMesh;
    using utility::nusight::graph;
    using GreenHorizonMsg = message::vision::GreenHorizon;

    GreenHorizonDetector::GreenHorizonDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("GreenHorizonDetector.yaml").then([this](const Configuration& config) {
            // Use configuration here from file GreenHorizonDetector.yaml
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.confidence_threshold = config["confidence_threshold"].as<double>();
            cfg.cluster_points       = config["cluster_points"].as<uint>();
            cfg.distance_offset      = config["distance_offset"].as<double>();
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

            // Partition the indices such that we only have the field points that dont have field surrounding them
            auto boundary = utility::vision::visualmesh::partition_points(
                indices.begin(),
                indices.end(),
                neighbours,
                [&](const int& idx) {
                    return cls(FIELD_INDEX, idx) + cls(LINE_INDEX, idx) >= cfg.confidence_threshold;
                });

            // Discard indices that are not on the boundary
            indices.resize(std::distance(indices.begin(), boundary));

            // Graph the remaining points if debugging
            if (log_level <= NUClear::DEBUG) {
                for (int idx : indices) {
                    emit(graph("Field point", rPWw.col(idx).x(), rPWw.col(idx).y()));
                }
            }

            std::vector<std::vector<int>> clusters;
            utility::vision::visualmesh::cluster_points(indices.begin(),
                                                        indices.end(),
                                                        neighbours,
                                                        cfg.cluster_points,
                                                        clusters);

            log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));

            // Lambda to get the closest distance of a cluster
            auto get_closest_distance = [&](const std::vector<int>& cluster) {
                // Find the cluster that is closest to the robot
                int closest_point = 0;
                for (const auto& idx : cluster) {
                    if (rPWw.col(closest_point).norm() > rPWw.col(idx).norm()) {
                        closest_point = idx;
                    }
                }
                return rPWw.col(closest_point).norm();
            };

            // Set the closest cluster to the first cluster
            int closest_cluster     = 0;
            double closest_distance = get_closest_distance(clusters[0]);

            // Find the cluster that is closest to the robot
            for (int i = 1; i < clusters.size(); i++) {
                double distance = get_closest_distance(clusters[i]);
                if (distance < closest_distance) {
                    closest_cluster  = i;
                    closest_distance = distance;
                }
            }

            auto field_cluster = clusters[closest_cluster];

            // Graph the remaining points if debugging
            if (log_level <= NUClear::DEBUG) {
                for (int idx : field_cluster) {
                    emit(graph("Field cluster point", rPWw.col(idx).x(), rPWw.col(idx).y()));
                }
            }

            // The remaining points are on the boundary of the field
            // They may also appear around other objects such as the ball
            log<NUClear::DEBUG>(fmt::format("Found {} points on the boundary of the field to create a convex hull with",
                                            field_cluster.size()));

            // Convex hull algorithms require at least three points
            if (field_cluster.size() < 3) {
                log<NUClear::DEBUG>("Not enough points to make a convex hull");
                return;
            }

            // Eigen::Matrix<double, 3, Eigen::Dynamic> points(3, 10);
            // // Create a scatter of point representing a square
            // points.col(0) << 0, 0, 0;
            // points.col(1) << 0, 1, 0;
            // points.col(2) << 1, 1, 0;
            // points.col(3) << 1, 0, 0;
            // points.col(4) << 0, 0.5, 0;
            // points.col(5) << 0.5, 1, 0;
            // points.col(6) << 1, 0.5, 0;
            // points.col(7) << 0.5, 0, 0;
            // points.col(8) << 0.5, 0.5, 0;
            // points.col(9) << 0.5, 0.3, 0;

            // Triangle test
            // points.col(0) << 0, 0, 0;
            // points.col(1) << 0.4, 0.8, 0;
            // points.col(2) << 0.1, 0.8, 0;
            // points.col(3) << 0.2, 0.4, 0;
            // points.col(4) << 0, 1, 0;
            // points.col(5) << 0.0, 0.5, 0;
            // points.col(6) << 0.5, 0.5, 0;
            // points.col(7) << 1, 1, 0;
            // points.col(8) << 0.7, 0.9, 0;
            // points.col(9) << 0.1, 0.1, 0;

            // points.col(0) << 0, 0, 0;
            // points.col(1) << 0.4, 0.8, 0;
            // points.col(2) << 0.35, -0.15, 0;
            // points.col(3) << 0.2, 0.4, 0;
            // points.col(4) << 0, 1, 0;
            // points.col(5) << 0.0, 0.5, 0;
            // points.col(6) << 0.5, 0.5, 0;
            // points.col(7) << 1, 1, 0;
            // points.col(8) << 0.7, 0.9, 0;
            // points.col(9) << 0.1, 0.1, 0;


            // std::vector<int> indices2(points.cols());
            // std::iota(indices2.begin(), indices2.end(), 0);

            // for (int idx : indices2) {
            //     emit(graph("Field point", points.col(idx).x(), points.col(idx).y()));
            // }

            log<NUClear::DEBUG>("Calculating convex hull");

            // Find the convex hull of the field points
            auto hull_indices = utility::math::geometry::chans_convex_hull(field_cluster, rPWw);
            // auto hull_indices = utility::math::geometry::chans_convex_hull(indices2, points);

            log<NUClear::DEBUG>("Calculated a convex hull");

            for (int idx : hull_indices) {
                emit(graph("Convex hull point", rPWw.col(idx).x(), rPWw.col(idx).y()));
            }

            auto msg = std::make_unique<GreenHorizonMsg>();

            // Preserve mesh so that anyone using the GreenHorizon can access the original data
            msg->mesh = const_cast<VisualMesh*>(&mesh)->shared_from_this();

            msg->id        = mesh.id;
            msg->Hcw       = mesh.Hcw;
            msg->timestamp = mesh.timestamp;
            msg->class_map = mesh.class_map;

            if (mesh.vision_ground_truth.exists) {
                msg->vision_ground_truth = mesh.vision_ground_truth;
            }

            // Find the convex hull of the cluster
            msg->horizon.reserve(hull_indices.size());
            for (const auto& idx : hull_indices) {
                const Eigen::Vector3d ray      = uPCw.col(idx);
                const double d                 = mesh.Hcw(2, 3) / ray.z();
                Eigen::Vector3d ray_projection = ray * d;
                const double norm              = ray_projection.head<2>().norm();
                ray_projection.head<2>() *= 1.0f + cfg.distance_offset / norm;
                msg->horizon.emplace_back(ray_projection.normalized().cast<float>());
            }
            log<NUClear::DEBUG>(fmt::format("Calculated a convex hull with {} points from a boundary with {} points",
                                            hull_indices.size(),
                                            field_cluster.size()));
            emit(std::move(msg));
        });
    }
}  // namespace module::vision
