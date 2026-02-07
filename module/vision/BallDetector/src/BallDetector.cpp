/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
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

#include "BallDetector.hpp"

#include <Eigen/Geometry>
#include <cstddef>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <numeric>
#include <optional>
#include <set>

#include "extension/Configuration.hpp"

#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/math/geometry/ConvexHull.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

// Make a formatter for Eigen::Transpose type so fmt::format know how to deal with it
template <typename Derived>
struct fmt::formatter<Eigen::Transpose<Derived>> : fmt::ostream_formatter {};

namespace module::vision {

    using extension::Configuration;

    using message::support::FieldDescription;
    using message::vision::Ball;
    using message::vision::Balls;
    using message::vision::GreenHorizon;

    using utility::math::coordinates::cartesianToReciprocalSpherical;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::nusight::graph;
    using utility::support::Expression;

    BallDetector::BallDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("BallDetector.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.confidence_threshold  = config["confidence_threshold"].as<double>();
            cfg.cluster_points        = config["cluster_points"].as<int>();
            cfg.merge_buffer_scalar   = config["merge_buffer_scalar"].as<double>();
            cfg.fill_requirement      = config["fill_requirement"].as<double>();
            cfg.minimum_ball_distance = config["minimum_ball_distance"].as<double>();
            cfg.distance_disagreement = config["distance_disagreement"].as<double>();
            cfg.maximum_deviation     = config["maximum_deviation"].as<double>();
            cfg.ball_angular_cov      = Eigen::Vector3d(config["ball_angular_cov"].as<Expression>());
        });

        on<Trigger<GreenHorizon>, With<FieldDescription>, Buffer<2>>().then(
            "Visual Mesh",
            [this](const GreenHorizon& horizon, const FieldDescription& field) {
                // Convenience variables
                const auto& cls        = horizon.mesh->classifications;
                const auto& neighbours = horizon.mesh->neighbourhood;
                // Unit vectors from camera to a point in the mesh, in world space
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& uPCw = horizon.mesh->uPCw.cast<double>();
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& rPWw = horizon.mesh->rPWw.cast<double>();
                const int BALL_INDEX                                 = horizon.class_map.at("ball");

                // PARTITION INDICES AND CLUSTER

                // Get some indices to partition
                std::vector<int> indices(horizon.mesh->indices.size());
                std::iota(indices.begin(), indices.end(), 0);

                // Partition the indices such that the ball points that have ball points surrounding them are removed,
                // and then resize the vector to remove those points
                auto boundary = utility::vision::visualmesh::partition_points(
                    indices.begin(),
                    indices.end(),
                    neighbours,
                    [&](const int& idx) {
                        return idx == int(indices.size()) || (cls(BALL_INDEX, idx) >= cfg.confidence_threshold);
                    });
                indices.resize(std::distance(indices.begin(), boundary));

                log<DEBUG>(fmt::format("Partitioned {} points", indices.size()));

                // Cluster all points into ball candidates
                // Points are clustered based on their connectivity to other ball points
                // Clustering is done in three steps
                // 1) Take the set of ball points found above and partition them into potential clusters by
                //    a) Add the first point and its ball neighbours to a cluster
                //    b) Find all other ball points who are neighbours of the points in the cluster
                //    c) Partition all of the indices that are in the cluster
                //    d) Repeat a-c for all points that were not partitioned
                //    e) Delete all partitions smaller than a given threshold
                // 2) Create additional clusters when multiple clusters are likely on the same ball by
                //    a) Precompute central axis and angular radius of each cluster
                //    b) Create an adjacency table with connections being minimal angular offset between clusters
                //    c) Find connected components to use as candidates for merging through depth first search
                //    d) For each connected component, add as a new cluster if it creates a more spherical cluster
                // 3) Discard all clusters are entirely above the green horizon

                std::vector<std::vector<int>> clusters;

                utility::vision::visualmesh::cluster_points(indices.begin(),
                                                            indices.end(),
                                                            neighbours,
                                                            cfg.cluster_points,
                                                            clusters);

                log<DEBUG>(fmt::format("Found {} clusters", clusters.size()));

                const size_t num_unmerged_clusters{clusters.size()};

                // FIND CENTRAL AXIS OF EACH CLUSTER
                // list of unit vectors from camera to cluster central axis in world space
                std::vector<Eigen::Vector3d> cluster_axes(num_unmerged_clusters);
                for (size_t i{}; i < num_unmerged_clusters; ++i) {
                    const auto& cluster = clusters[i];

                    cluster_axes[i] = utility::vision::visualmesh::find_cluster_central_axis(cluster, uPCw);
                }

                // FIND ANGULAR RADIUS OF EACH CLUSTER
                // Find the ray (uPCw) with the greatest distance from the central axis (uBCw) to then determine the
                // largest angular radius possible from the edge points available. Equal to cos(theta), where theta
                // is the angle between the central ball axis (uBCw) and the edge of the ball.
                std::vector<double> cluster_radii(num_unmerged_clusters);
                for (size_t i{}; i < num_unmerged_clusters; ++i) {
                    const auto& cluster         = clusters[i];
                    const Eigen::Vector3d& uBCw = cluster_axes[i];

                    cluster_radii[i] = utility::vision::visualmesh::find_cluster_angular_radius(cluster, uPCw, uBCw);
                }

                // Create a adjacency matrix of potentially valid cluster merges
                std::vector<std::vector<size_t>> adj_matrix(num_unmerged_clusters);
                for (size_t i{}; i < num_unmerged_clusters; ++i) {
                    // convert cluster radius stored as cos(theta) to angles
                    double r_i_angle = std::acos(cluster_radii[i]);
                    for (size_t j{i + 1}; j < num_unmerged_clusters; ++j) {
                        double r_j_angle = std::acos(cluster_radii[j]);

                        // If angular separation of 2 cluster axes < sum of their angular radii, from the camera's view
                        // they overlap and can be merged. In use, sum of radii is multiplied by merge_buffer_scalar
                        // and this new combined radius is used for checking the overlap. A merge_buffer_scalar of 1.2
                        // allows merging of clusters with a gap of < 0.2 * sum of radii between their original circles

                        // Find angular offset between axis of cluster using the dot product formula
                        double angular_offset{std::acos(cluster_axes[i].dot(cluster_axes[j]))};
                        double offset_allowance{(r_i_angle + r_j_angle) * (cfg.merge_buffer_scalar)};

                        if (angular_offset < offset_allowance) {
                            adj_matrix[i].push_back(j);
                            adj_matrix[j].push_back(i);
                        }
                    }
                }

                // keep vector of sphericality scores for reuse, these are only of merged clusters
                std::vector<double> merged_cluster_circular_fit;

                // Find connected components (merge candidates) through depth first search
                std::vector<char> visited(num_unmerged_clusters, false);
                std::vector<char> merged(num_unmerged_clusters, false);

                // Reuse vectors in each loop
                std::vector<size_t> stack;
                stack.reserve(num_unmerged_clusters);
                std::vector<size_t> mergeable_cluster_indices;
                mergeable_cluster_indices.reserve(num_unmerged_clusters);
                for (size_t i{}; i < num_unmerged_clusters; ++i) {

                    if (visited[i]) {
                        continue;
                    }

                    visited[i] = true;
                    stack.push_back(i);

                    // Clear indices again to allow reuse of the mergeable indices vector
                    // The stack is effectively cleared each loop as DFS can only end with the stack empty
                    mergeable_cluster_indices.clear();

                    while (!stack.empty()) {
                        size_t current = stack.back();
                        stack.pop_back();

                        mergeable_cluster_indices.push_back(current);

                        for (size_t neighbour : adj_matrix[current]) {
                            if (!visited[neighbour]) {
                                stack.push_back(neighbour);
                                visited[neighbour] = true;
                            }
                        }
                    }

                    // End early if there aren't multiple clusters to merge
                    if (mergeable_cluster_indices.size() <= 1) {
                        continue;
                    }

                    // Look at the combination of balls with the closest fit to a circle to avoid merging a ball with a
                    // nearby false detection. Greedily look at only the cases nCn and nCn-1 to minimise checked cases
                    // as multiple nearby false detections are unlikely.

                    // vector from camera to centre of the merged cluster
                    Eigen::Vector3d rBCw = Eigen::Vector3d::Zero();
                    for (size_t i : mergeable_cluster_indices) {
                        std::vector<int>& cluster = clusters[i];
                        for (int j : cluster) {
                            rBCw += uPCw.col(j);
                        }
                    }

                    // It is a list of unit vectors uBCw from camera to centre of the ball in world space, where
                    // this ball is composed of a cluster made from all mergeable_cluster_indices except
                    // mergeable_cluster_indices[n] where n is the index in this vector
                    std::vector<Eigen::Vector3d> skipped_cluster_axes;
                    skipped_cluster_axes.reserve(mergeable_cluster_indices.size());

                    for (size_t i{}; i < mergeable_cluster_indices.size(); ++i) {
                        // this is the case where clusters[mergeable_cluster_indices[i]] is skipped

                        // the central axis of this cluster can be found through copying rBCw, then removing all of
                        // the vectors this cluster added to it consists of to reuse calculations
                        Eigen::Vector3d nBCw_skipped = rBCw;
                        std::vector<int>& cluster    = clusters[mergeable_cluster_indices[i]];
                        for (int idx : cluster) {
                            nBCw_skipped -= uPCw.col(idx);
                        }
                        nBCw_skipped.normalize();
                        skipped_cluster_axes.push_back(nBCw_skipped);
                    }

                    // Equal to cos(theta), where theta is the angle between the central ball axis (uBCw) and the
                    // edge of the ball. The ball is a cluster made from all mergeable_cluster_indices except
                    // mergeable_cluster_indices[n] where n is the index in this vector
                    std::vector<double> skipped_cluster_radii;
                    skipped_cluster_radii.reserve(mergeable_cluster_indices.size());
                    for (size_t i{}; i < mergeable_cluster_indices.size(); ++i) {
                        // radius is the max angular offset between any detected point (uPCw) and the axis (uBCw)
                        double radius{1.0};
                        for (size_t j{}; j < mergeable_cluster_indices.size(); ++j) {
                            if (i == j) {
                                // this cluster is skipped
                                continue;
                            }
                            std::vector<int>& cluster = clusters[mergeable_cluster_indices[j]];

                            // get the maximum angular radius for this cluster
                            double new_radius =
                                utility::vision::visualmesh::find_cluster_angular_radius(cluster,
                                                                                         uPCw,
                                                                                         skipped_cluster_axes[i]);

                            // a smaller radius is a larger angular offset as radius is cos(theta)
                            radius = new_radius < radius ? new_radius : radius;
                        }
                        skipped_cluster_radii.push_back(radius);
                    }

                    std::vector<double> skipped_cluster_circular_fit;
                    skipped_cluster_circular_fit.reserve(mergeable_cluster_indices.size());
                    for (size_t i{}; i < mergeable_cluster_indices.size(); ++i) {

                        // temp thing for now testing

                        // Goodness of fit to a ball is measured in number of points observed divided by the number of
                        // expected points the proposed ball would contain
                        std::vector<std::vector<int>*> points;
                        size_t observed_bounded{};
                        for (size_t j{}; j < mergeable_cluster_indices.size(); ++j) {
                            if (i == j) {
                                continue;
                            }

                            points.push_back(&clusters[mergeable_cluster_indices[j]]);
                            observed_bounded += clusters[mergeable_cluster_indices[j]].size();
                        }

                        const size_t expected_bounded{
                            utility::vision::visualmesh::find_number_bounded_points(points,
                                                                                    neighbours,
                                                                                    skipped_cluster_axes[i],
                                                                                    skipped_cluster_radii[i],
                                                                                    uPCw)};
                        const double filled_amount{static_cast<double>(observed_bounded) / expected_bounded};

                        skipped_cluster_circular_fit.push_back(filled_amount);
                    }

                    // Calculations for the nCn clusters option
                    Eigen::Vector3d uBCw = rBCw.normalized();
                    double radius{1.0};
                    for (size_t index : mergeable_cluster_indices) {
                        std::vector<int>& cluster = clusters[index];
                        double new_radius{
                            utility::vision::visualmesh::find_cluster_angular_radius(cluster, uPCw, uBCw)};

                        radius = new_radius < radius ? new_radius : radius;
                    }

                    std::vector<std::vector<int>*> points;
                    size_t observed_bounded{};
                    for (size_t index : mergeable_cluster_indices) {
                        points.push_back(&clusters[index]);
                        observed_bounded += clusters[index].size();
                    }

                    const size_t expected_bounded{utility::vision::visualmesh::find_number_bounded_points(points,
                                                                                                          neighbours,
                                                                                                          uBCw,
                                                                                                          radius,
                                                                                                          uPCw)};
                    const double filled_amount{static_cast<double>(observed_bounded) / expected_bounded};

                    std::optional<size_t> removed_indice{std::nullopt};

                    // keep best fill as the unmerged option at the start
                    double best_fill{filled_amount};
                    for (size_t i{}; i < skipped_cluster_circular_fit.size(); ++i) {
                        if (skipped_cluster_circular_fit[i] > best_fill) {
                            best_fill      = skipped_cluster_circular_fit[i];
                            removed_indice = i;
                        }
                    }

                    // remove the indice if its going to give a better spherical fit
                    if (removed_indice.has_value()) {
                        mergeable_cluster_indices.erase(mergeable_cluster_indices.begin() + removed_indice.value());

                        // skip merging if its better to do no merges
                        if (mergeable_cluster_indices.size() <= 1) {
                            log<DEBUG>("Merge rejected due to worse sphericality");
                            continue;
                        }

                        // TODO maybe have this show on nusight
                        log<DEBUG>("Merged ball created and one possible cluster was removed when making it");
                    }

                    clusters.emplace_back();
                    std::vector<int>& cluster = clusters.back();

                    // Reserve memory in advance, and note those indices as used for merging
                    size_t num_points{};
                    for (size_t i : mergeable_cluster_indices) {
                        num_points += clusters[i].size();
                        merged[i] = true;
                    }
                    cluster.reserve(num_points);

                    // Copy points from other clusters into the merged one
                    for (size_t i : mergeable_cluster_indices) {
                        cluster.insert(cluster.end(), clusters[i].begin(), clusters[i].end());
                    }

                    cluster_axes.push_back(removed_indice.has_value() ? skipped_cluster_axes[removed_indice.value()]
                                                                      : uBCw);
                    cluster_radii.push_back(removed_indice.has_value() ? skipped_cluster_radii[removed_indice.value()]
                                                                       : radius);
                    merged_cluster_circular_fit.push_back(best_fill);
                }

                log<DEBUG>(fmt::format("Created {} additional clusters through merging {} clusters",
                                       clusters.size() - num_unmerged_clusters,
                                       std::count(merged.begin(), merged.end(), true)));

                // Partition the clusters such that clusters above the green horizons are removed,
                // and then resize the vector to remove them
                auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(clusters,
                                                                                            horizon.horizon,
                                                                                            rPWw,
                                                                                            false,
                                                                                            true,
                                                                                            true);
                clusters.resize(std::distance(clusters.begin(), green_boundary));

                log<DEBUG>(fmt::format("Found {} clusters below green horizon", clusters.size()));

                // Create the Balls message, which will contain a Ball for every cluster that is a valid ball
                auto balls = std::make_unique<Balls>();

                // Balls is still emitted even if there are no balls, to indicate to other modules that no balls are
                // currently visible
                if (clusters.empty()) {
                    log<DEBUG>("Found no balls.");
                    emit(std::move(balls));
                    return;
                }

                // Reserve the memory in advance for efficiency
                balls->balls.reserve(clusters.size());

                balls->id        = horizon.id;         // camera id
                balls->timestamp = horizon.timestamp;  // time when the image was taken
                balls->Hcw       = horizon.Hcw;        // world to camera transform at the time the image was taken

                // World to camera transform, to be used in for loop below
                const Eigen::Isometry3d Hcw(horizon.Hcw.cast<double>());

                // CHECK EACH CLUSTER FOR VALID BALL
                for (size_t i{}; i < clusters.size(); ++i) {
                    const auto& cluster = clusters[i];
                    Ball b;

                    // Find central axis of ball, using precomputed measurements from merge logic
                    // uBCw: unit vector from camera to ball central axis in world space
                    Eigen::Vector3d uBCw{cluster_axes[i]};

                    // Find angular radius of ball from camera, where radius is = cos(theta)
                    double radius{cluster_radii[i]};

                    // The vectors are in world space, multiply by Rcw to get the central axis in camera space
                    b.uBCc = Hcw.rotation() * uBCw;
                    // Angular radius, or cos(theta) where theta is the angle between central axis and ball edge
                    b.radius = radius;

                    // CALCULATE DISTANCE TO BALL WITH TWO METHODS
                    // 1. Angular-based distance
                    // 2. Projection-based distance

                    // Angular-based distance
                    // https://en.wikipedia.org/wiki/Angular_diameter
                    // From the link, the formula with arcsin is used since a ball is a spherical object
                    // The variables are:
                    //      delta: 2 * arccos(radius)
                    //      d_act: 2 * field.ball_radius
                    //      D: distance
                    // Rearranging the equation gives
                    //      distance = (field.ball_radius) / (sin(arccos(radius)))
                    // Using sin(arccos(x)) = sqrt(1 - x^2)
                    //      distance = field.ball_radius / sqrt(1 - radius^2)
                    const double angular_distance = field.ball_radius / std::sqrt(1.0 - radius * radius);
                    // Convert uBCc into a position vector (rBCc) and then convert it into Spherical Reciprocal
                    // Coordinates (1/distance, phi, theta)
                    b.measurements.emplace_back();
                    b.measurements.back().type       = Ball::MeasurementType::ANGULAR;
                    b.measurements.back().rBCc       = b.uBCc * angular_distance;
                    b.measurements.back().covariance = cfg.ball_angular_cov.asDiagonal();

                    // Projection-based distance
                    // Given a flat X-Y plane that intersects through the middle of the ball, find the point where
                    // the central axis vector (uBCw) intersects with this plane. Get the distance from the camera to
                    // this point. https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection#Algebraic_form
                    //      Plane normal: (0, 0, 1)
                    //      Point on plane: (0, 0, rWCw.z() + field.ball_radius)
                    //      Line direction: uBCw
                    //      Point on line: (0, 0, 0)
                    // Since the plane normal zeros out x and y, only consider z
                    // rWCw = -Hcw.inverse().translation().z()
                    const double projection_distance = (field.ball_radius - Hcw.inverse().translation().z()) / uBCw.z();
                    // Create a ball measurement message for this calculation
                    b.measurements.emplace_back();
                    b.measurements.back().type       = Ball::MeasurementType::PROJECTION;
                    b.measurements.back().rBCc       = b.uBCc * projection_distance;
                    b.measurements.back().covariance = cfg.ball_angular_cov.asDiagonal();


                    /***********************************************
                     *                  THROWOUTS                  *
                     ***********************************************/

                    // For this particular ball, see if it should be thrown out
                    // For debugging purposes, go through each check
                    log<DEBUG>("**************************************************");
                    log<DEBUG>("*                    THROWOUTS                   *");
                    log<DEBUG>("**************************************************");
                    bool keep = true;
                    b.colour.fill(1.0);  // a valid ball has a white colour in NUsight

                    // DISCARD IF CLUSTER WAS USED FOR MERGING
                    // Merged is the same size as number of unmerged clusters and merged clusters are placed after
                    // unmerged, so if i >= merged.size() it was made not used in merging, and the check prevents a
                    // out of bounds read in the case a merge has been made making clusters.size() > merged.size()

                    if (i < merged.size() && merged[i]) {

                        log<DEBUG>("Ball discarded: merged with another detection to create another ball");
                        log<DEBUG>("--------------------------------------------------");
                        // Clusters that were used for merging will show up as orange in NUsight
                        b.colour = keep ? message::conversion::math::vec4(1.0, 0.65, 0.0, 1.0) : b.colour;
                        keep     = false;
                    }

                    // DISCARD IF FILL IS LOWER THAN FILL REQUIREMENT - CALCULATE DEGREE OF FIT TO CIRCLE
                    // Degree of fit to a circle is defined as n. observed points on the ball / n. expected
                    // ball points on the visual mesh, for a ball of that centre and radius
                    double fill_amount;
                    if (i >= num_unmerged_clusters) {
                        // use precomputed circular fit from merging logic
                        fill_amount = merged_cluster_circular_fit[i - num_unmerged_clusters];
                    }
                    else {
                        fill_amount = static_cast<double>(cluster.size())
                                      / utility::vision::visualmesh::find_number_bounded_points(cluster,
                                                                                                neighbours,
                                                                                                uBCw,
                                                                                                radius,
                                                                                                uPCw);
                    }

                    if (fill_amount < cfg.fill_requirement) {
                        log<DEBUG>(fmt::format("Ball discarded: fill_amount ({}) < fill_requirement ({})",
                                               fill_amount,
                                               cfg.fill_requirement));
                        log<DEBUG>("--------------------------------------------------");
                        // Balls that violate degree of fit will show as green in NUsight
                        b.colour = keep ? message::conversion::math::vec4(0.0, 1.0, 0.0, 1.0) : b.colour;
                        keep     = false;
                    }

                    // DISCARD IF PROJECTION_DISTANCE AND ANGULAR_DISTANCE ARE TOO FAR APART
                    const double max_distance = std::max(projection_distance, angular_distance);

                    if ((std::abs(projection_distance - angular_distance) / max_distance) > cfg.distance_disagreement) {

                        log<DEBUG>(
                            fmt::format("Ball discarded: Width and proj distance disagree too much: width = "
                                        "{}, proj = {}",
                                        angular_distance,
                                        projection_distance));
                        log<DEBUG>("--------------------------------------------------");
                        // Balls that violate this but not previous checks will show as blue in NUsight
                        b.colour = keep ? message::conversion::math::vec4(0.0, 0.0, 1.0, 1.0) : b.colour;
                        keep     = false;
                    }

                    // DISCARD IF THE DISTANCE FROM THE BALL TO THE ROBOT IS TOO CLOSE
                    // Prevents the robot itself being classed as a ball, ie its arms/hands
                    if (angular_distance < cfg.minimum_ball_distance) {

                        log<DEBUG>(fmt::format("Ball discarded: distance ({}) < minimum_ball_distance ({})",
                                               angular_distance,
                                               cfg.minimum_ball_distance));
                        log<DEBUG>("--------------------------------------------------");
                        // Balls that violate this but not previous checks will show as red in NUsight
                        b.colour = keep ? message::conversion::math::vec4(1.0, 0.0, 0.0, 1.0) : b.colour;
                        keep     = false;
                    }

                    // DISCARD IF THE BALL IS FURTHER THAN THE LENGTH OF THE FIELD
                    if (angular_distance > field.dimensions.field_length) {

                        log<DEBUG>(
                            fmt::format("Ball discarded: Distance to ball greater than field length: distance = "
                                        "{}, field length = {}",
                                        angular_distance,
                                        field.dimensions.field_length));
                        log<DEBUG>("--------------------------------------------------");
                        // Balls that violate this but not previous checks will show as yellow in NUsight
                        b.colour = keep ? message::conversion::math::vec4(1.0, 0.0, 1.0, 1.0) : b.colour;
                        keep     = false;
                    }

                    log<DEBUG>(fmt::format("Camera {}", balls->id));
                    log<DEBUG>(fmt::format("radius {}", b.radius));
                    log<DEBUG>(fmt::format("Axis {}", b.uBCc.transpose()));
                    log<DEBUG>(
                        fmt::format("Distance {} - rBCc {}", angular_distance, b.measurements[0].rBCc.transpose()));
                    log<DEBUG>(fmt::format("Projection Distance {} - rBCc {}",
                                           projection_distance,
                                           b.measurements[1].rBCc.transpose()));
                    log<DEBUG>(fmt::format("Distance Throwout {}",
                                           std::abs(projection_distance - angular_distance) / max_distance));
                    // If the index of the cluster is larger than the number of unmerged clusters
                    // It was made by merging as combined clusters are appended after unmerged ones
                    log<DEBUG>(fmt::format("Created through merging clusters: {}", i >= num_unmerged_clusters));
                    log<DEBUG>("**************************************************");

                    if (!keep && log_level > DEBUG) {
                        b.measurements.clear();
                    }

                    // If the ball passed the checks, add it to the Balls message to be emitted
                    // If it didn't pass the checks, but we're debugging, then emit the ball to see throwouts in NUsight
                    if (keep || log_level <= DEBUG) {
                        balls->balls.push_back(std::move(b));
                    }

                    if (horizon.vision_ground_truth.exists) {
                        Eigen::Affine3d Hcw(horizon.Hcw);

                        const Eigen::Vector3d rBCc = Hcw * horizon.vision_ground_truth.rBWw.cast<double>();
                        const Eigen::Vector3d rBWw = horizon.vision_ground_truth.rBWw.cast<double>();

                        Eigen::Vector3d ball_position_projection = uBCw * projection_distance;
                        Eigen::Vector3d ball_position_angular    = uBCw * angular_distance;
                        Eigen::Vector3d ball_error_projection    = (ball_position_projection - rBCc).cwiseAbs();
                        Eigen::Vector3d ball_error_angular       = (ball_position_angular - rBCc).cwiseAbs();

                        emit(graph("True rBCc", rBCc.x(), rBCc.y(), rBCc.z()));
                        emit(graph("True rBWw", rBWw.x(), rBWw.y(), rBWw.z()));
                        emit(graph("Angular Distance Ball error",
                                   ball_error_projection.x(),
                                   ball_error_projection.y(),
                                   ball_error_projection.z()));
                        emit(graph("Projection Distance Ball error",
                                   ball_error_angular.x(),
                                   ball_error_angular.y(),
                                   ball_error_angular.z()));
                    }
                }

                // Emit the Balls message, even if no balls were found
                emit(std::move(balls));
            });
    }
}  // namespace module::vision
