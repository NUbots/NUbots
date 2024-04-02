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
#include "FieldLineDetector.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <numeric>

#include "extension/Configuration.hpp"

#include "message/vision/FieldLines.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"


namespace module::vision {

    using extension::Configuration;
    using message::vision::FieldLines;
    using message::vision::GreenHorizon;

    FieldLineDetector::FieldLineDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FieldLineDetector.yaml").then([this](const Configuration& config) {
            log_level                = config["log_level"].as<NUClear::LogLevel>();
            cfg.confidence_threshold = config["confidence_threshold"].as<double>();
            cfg.cluster_points       = config["cluster_points"].as<int>();
        });

        on<Trigger<GreenHorizon>, Buffer<2>>().then("Field Line Detector", [this](const GreenHorizon& horizon) {
            // Convenience variables
            const auto& cls                                      = horizon.mesh->classifications;
            const auto& neighbours                               = horizon.mesh->neighbourhood;
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& rPWw = horizon.mesh->rPWw.cast<double>();
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& uPCw = horizon.mesh->uPCw.cast<double>();
            const int LINE_INDEX                                 = horizon.class_map.at("line");

            // PARTITION INDICES AND CLUSTER
            // Get some indices to partition
            std::vector<int> indices(horizon.mesh->indices.size());
            std::iota(indices.begin(), indices.end(), 0);

            // Partition the indices such that the field line points that have field line points surrounding them are
            // removed, and then resize the vector to remove those points
            auto boundary = utility::vision::visualmesh::partition_points(
                indices.begin(),
                indices.end(),
                neighbours,
                [&](const int& idx) {
                    return idx == int(indices.size()) || (cls(LINE_INDEX, idx) >= cfg.confidence_threshold);
                });
            indices.resize(std::distance(indices.begin(), boundary));
            log<NUClear::DEBUG>(fmt::format("Partitioned {} points", indices.size()));

            // Cluster the field lines
            std::vector<std::vector<int>> clusters;
            utility::vision::visualmesh::cluster_points(indices.begin(),
                                                        indices.end(),
                                                        neighbours,
                                                        cfg.cluster_points,
                                                        clusters);
            log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));

            // Partition the clusters such that clusters above the green horizons are removed,
            // and then resize the vector to remove them
            auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(clusters,
                                                                                        horizon.horizon,
                                                                                        rPWw,
                                                                                        false,
                                                                                        true,
                                                                                        false);
            clusters.resize(std::distance(clusters.begin(), green_boundary));
            log<NUClear::DEBUG>(fmt::format("Found {} clusters below green horizon", clusters.size()));

            // The FieldLines message contains a FieldLine for every cluster that is a valid field
            // line
            auto lines       = std::make_unique<FieldLines>();
            lines->id        = horizon.id;         // camera id
            lines->timestamp = horizon.timestamp;  // time when the image was taken
            lines->Hcw       = horizon.Hcw;        // world to camera transform at the time the image was taken

            for (auto& cluster : clusters) {
                for (const auto& idx : cluster) {
                    lines->points.push_back(uPCw.col(idx));
                    lines->rPWw.push_back(rPWw.col(idx));
                }
            }

            emit(lines);
        });
    }

}  // namespace module::vision
