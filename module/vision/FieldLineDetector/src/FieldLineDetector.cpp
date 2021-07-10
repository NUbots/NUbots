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

#include "FieldLineDetector.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <numeric>

#include "extension/Configuration.hpp"

#include "message/support/FieldDescription.hpp"
#include "message/vision/FieldLine.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/math/geometry/ConvexHull.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::support::FieldDescription;
    using message::vision::FieldLine;
    using message::vision::FieldLines;
    using message::vision::GreenHorizon;

    using utility::math::coordinates::cartesianToSpherical;
    using utility::support::Expression;

    FieldLineDetector::FieldLineDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // Trigger the same function when either update
        on<Configuration>("FieldLineDetector.yaml").then([this](const Configuration& cfg) {
            config.confidence_threshold = cfg["confidence_threshold"].as<float>();
            config.cluster_points       = cfg["cluster_points"].as<int>();
            config.line_cov             = Eigen::Vector3f(cfg["line_cov"].as<Expression>()).asDiagonal();
            config.debug                = cfg["debug"].as<bool>();
        });

        on<Trigger<GreenHorizon>, With<FieldDescription>, Buffer<2>>().then(
            "Field Line Detector",
            [this](const GreenHorizon& horizon, const FieldDescription& field) {
                // Convenience variables
                const auto& cls                                     = horizon.mesh->classifications;
                const auto& neighbours                              = horizon.mesh->neighbourhood;
                const Eigen::Matrix<float, 3, Eigen::Dynamic>& rays = horizon.mesh->rays;
                const float world_offset                            = std::atan2(horizon.Hcw(0, 1), horizon.Hcw(0, 0));
                const int FIELD_LINE_INDEX                          = horizon.class_map.at("line");

                // Get some indices to partition
                std::vector<int> indices(horizon.mesh->indices.size());
                std::iota(indices.begin(), indices.end(), 0);

                // Partition the indices such that we only have the field line points that dont have field line
                // surrounding them
                auto boundary = utility::vision::visualmesh::partition_points(
                    indices.begin(),
                    indices.end(),
                    neighbours,
                    [&](const int& idx) {
                        return idx == int(indices.size())
                               || (cls(FIELD_LINE_INDEX, idx) >= config.confidence_threshold);
                    });

                // Discard indices that are not on the boundary and are not below the green horizon
                indices.resize(std::distance(indices.begin(), boundary));

                // Cluster all points into field line candidates
                // Points are clustered based on their connectivity to other field line points
                // Clustering is down in two steps
                // 1) We take the set of field line points found above and partition them into potential clusters by
                //    a) Add the first point and its field line neighbours to a cluster
                //    b) Find all other field line points who are neighbours of the points in the cluster
                //    c) Partition all of the indices that are in the cluster
                //    d) Repeat a-c for all points that were not partitioned
                //    e) Delete all partitions smaller than a given threshold
                // 2) Discard all clusters that are not below the green horizon
                std::vector<std::vector<int>> clusters;
                utility::vision::visualmesh::cluster_points(indices.begin(),
                                                            indices.end(),
                                                            neighbours,
                                                            config.cluster_points,
                                                            clusters);


                log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));


                // Discard any clusters that are notentirely below the green horizon
                auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(clusters.begin(),
                                                                                            clusters.end(),
                                                                                            horizon.horizon.begin(),
                                                                                            horizon.horizon.end(),
                                                                                            rays,
                                                                                            false,
                                                                                            true);
                clusters.resize(std::distance(clusters.begin(), green_boundary));

                log<NUClear::DEBUG>(fmt::format("Found {} clusters that intersect the green horizon", clusters.size()));

                if (clusters.size() > 0) {
                    auto lines = std::make_unique<FieldLines>();

                    // Calculate potential number of points
                    int n_points = 0;
                    for (auto& cluster : clusters) {
                        n_points += cluster.size();
                    }
                    lines->lines.reserve(n_points);

                    lines->id                 = horizon.id;
                    lines->timestamp          = horizon.timestamp;
                    lines->Hcw                = horizon.Hcw;
                    const float camera_height = horizon.Hcw.cast<float>().inverse().translation().z();

                    for (auto& cluster : clusters) {

                        // For each point in the cluster
                        //     1) Project it to the observation plane
                        //     2) Get distance from camera to point on plane (measurement distance)
                        for (auto& point : cluster) {
                            FieldLine l;

                            // Calculate the vector from camera to field line point (rLCc)
                            const Eigen::Vector3f& axis = rays.col(point);
                            const float distance        = camera_height / std::abs(axis.z());
                            const Eigen::Vector3f rLCc  = axis * distance;

                            // Attach the measurement to the object (distance from camera to point on plane)
                            auto measurement = cartesianToSpherical(rLCc);
                            l.rLCc       = Eigen::Vector3f(1.0f / measurement.x(), measurement.y(), measurement.z());
                            l.covariance = config.line_cov.asDiagonal();

                            // Throwout
                            bool keep = true;

                            // IF THE POINT IS FURTHER THAN THE LENGTH OF THE FIELD
                            if (distance > field.dimensions.field_length) {

                                log<NUClear::DEBUG>(fmt::format(
                                    "Line Point discarded: Distance to point greater than field length: distance = "
                                    "{}, field length = {}",
                                    distance,
                                    field.dimensions.field_length));
                                log<NUClear::DEBUG>("--------------------------------------------------");

                                keep = false;
                            }

                            // Add field line point to message
                            if (keep) {
                                lines->lines.push_back(std::move(l));
                            }
                        }
                        emit(std::move(lines));
                    }
                }
            });
    }
}  // namespace module::vision
