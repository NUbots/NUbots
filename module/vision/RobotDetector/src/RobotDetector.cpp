/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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

#include "RobotDetector.hpp"

#include <algorithm>
#include <fmt/format.h>
#include <numeric>

#include "extension/Configuration.hpp"

#include "message/vision/GreenHorizon.hpp"
#include "message/vision/Robot.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::vision::GreenHorizon;
    using message::vision::Robot;
    using message::vision::Robots;
    using utility::math::coordinates::cartesianToReciprocalSpherical;


    RobotDetector::RobotDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("RobotDetector.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotDetector.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.confidence_threshold   = config["confidence_threshold"].as<double>();
            cfg.cluster_points         = config["cluster_points"].as<int>();
            cfg.minimum_robot_distance = config["minimum_robot_distance"].as<double>();
            cfg.robot_radius           = config["robot_radius"].as<double>();
            cfg.robot_covariance       = config["robot_covariance"].as<Expression>();
        });

        on<Trigger<GreenHorizon>, Buffer<2>>().then("Visual Mesh", [this](const GreenHorizon& horizon) {
            // Convenience variables
            const auto& cls        = horizon.mesh->classifications;
            const auto& neighbours = horizon.mesh->neighbourhood;
            // Unit vectors from camera to a point in the mesh, in world space
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& rPWw = horizon.mesh->rPWw.cast<double>();
            const int ROBOT_INDEX                                = horizon.class_map.at("robot");

            // Get indices to partition
            std::vector<int> indices(horizon.mesh->indices.size());
            std::iota(indices.begin(), indices.end(), 0);

            // Set up the check for if a point is a robot point
            auto is_robot = [&](const int& idx) { return cls(ROBOT_INDEX, idx) >= cfg.confidence_threshold; };

            // Partition the indices such that we only have the robot points
            auto robot_points = std::partition(indices.begin(), indices.end(), is_robot);
            indices.resize(std::distance(indices.begin(), robot_points));

            // Cluster the points, assuming each cluster is a robot
            std::vector<std::vector<int>> clusters;
            utility::vision::visualmesh::cluster_points(indices.begin(),
                                                        indices.end(),
                                                        neighbours,
                                                        cfg.cluster_points,
                                                        clusters);

            log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));

            // Get only the boundary of each cluster
            for (auto& cluster : clusters) {
                auto boundary =
                    utility::vision::visualmesh::boundary_points(cluster.begin(), cluster.end(), neighbours, is_robot);
                cluster.resize(std::distance(cluster.begin(), boundary));
            }

            // Partition the clusters such that clusters outside the hull are removed, but the ones that are inside or
            // intersect are kept
            auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(clusters,
                                                                                        horizon.horizon,
                                                                                        rPWw,
                                                                                        false,
                                                                                        true,
                                                                                        true);
            clusters.resize(std::distance(clusters.begin(), green_boundary));

            log<NUClear::DEBUG>(fmt::format("Found {} clusters below green horizon", clusters.size()));

            // Create the robot message that will hold all observed robots, with general camera details
            auto robots       = std::make_unique<Robots>();
            robots->timestamp = horizon.mesh->timestamp;
            robots->id        = horizon.mesh->id;
            robots->Hcw       = horizon.mesh->Hcw;

            // World to camera transform, to be used in for loop below
            const Eigen::Isometry3d Hwc(horizon.Hcw.inverse());

            // Process each robot cluster, and emit a message with each
            for (const auto& cluster : clusters) {
                // Create the robot message - only add to robots when confirmed to be okay
                Robot robot;

                // We assume the closest point to the camera is the base of the detected robot
                // Compare with rPCw
                auto closest_element =
                    std::min_element(cluster.begin(), cluster.end(), [&](const int& a, const int& b) {
                        return (rPWw.col(a) - Hwc.translation()).norm() < (rPWw.col(b) - Hwc.translation()).norm();
                    });

                // Transform the point so it is from the camera
                robot.rRCc = horizon.Hcw * rPWw.col(*closest_element);

                // Robots that are too close to us are unlikely to be other robots
                if (robot.rRCc.norm() < cfg.minimum_robot_distance) {
                    log<NUClear::DEBUG>(fmt::format("Cluster rejected due to distance: {}. Min allowed distance {}.",
                                                    robot.rRCc.norm(),
                                                    cfg.minimum_robot_distance));
                    continue;
                }

                // Should be changed to the average confidence of the robot
                robot.covariance = cfg.robot_covariance;
                robot.radius     = cfg.robot_radius;
                robots->robots.push_back(std::move(robot));
            }

            log<NUClear::DEBUG>(fmt::format("Found {} robots", robots->robots.size()));

            emit(robots);
        });
    }
}  // namespace module::vision
