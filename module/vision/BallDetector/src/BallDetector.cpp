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
#include <ranges>
#include <set>

#include "extension/Configuration.hpp"

#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/math/coordinates.hpp"
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
    using utility::vision::visualmesh::find_cluster_angular_radius;
    using utility::vision::visualmesh::find_cluster_central_axis;
    using utility::vision::visualmesh::find_cluster_circularity;

    namespace {
        struct BallCandidate {
            // Indices of the clusters used to make this cluster
            std::vector<size_t> cluster_indices;

            // Central axis of the ball
            Eigen::Vector3d uBCw = Eigen::Vector3d::Zero();

            // Radius is = cos(theta), where theta is angle between uBCw and the furthest point on the ball
            double radius = 0.0;

            bool used_in_merge{false};

            std::optional<double> circularity{std::nullopt};
        };
    }  // namespace

    BallDetector::BallDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("BallDetector.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.confidence_threshold  = config["confidence_threshold"].as<double>();
            cfg.cluster_points        = config["cluster_points"].as<int>();
            cfg.merge_buffer_scalar   = config["merge_buffer_scalar"].as<double>();
            cfg.merge_preference      = config["merge_preference"].as<double>();
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
                // 2) Create BallCandidates from clusters, considering opportunities to merge clusters by
                //    a) Create a ball candidate for each each cluster
                //    b) Compute central axis and angular radius of each ball candidate
                //    c) Create an adjacency table with connections being minimal angular offset between ball candidates
                //    d) For each connected component, check if merging nCn or nCn-1 ball candidates is preferred
                //    e) Create a new ball candidate from the merged candidates, then compute its radius and axis
                //    f) Mark candidates used for merging as used in merging
                // 3) Discard all ball candidates who's points are entirely above the green horizon

                std::vector<std::vector<int>> clusters{};
                utility::vision::visualmesh::cluster_points(indices.begin(),
                                                            indices.end(),
                                                            neighbours,
                                                            cfg.cluster_points,
                                                            clusters);

                log<DEBUG>(fmt::format("Found {} clusters", clusters.size()));

                // Create ball candidates for each cluster
                std::vector<BallCandidate> ball_candidates(clusters.size());
                for (size_t i = 0; i < clusters.size(); ++i) {
                    ball_candidates[i].cluster_indices.push_back(i);
                }

                // Find central axis of each ball candidate
                for (BallCandidate& ball : ball_candidates) {
                    // Each candidate corresponds to a cluster currently
                    const std::vector<int>& cluster = clusters[ball.cluster_indices[0]];
                    ball.uBCw                       = find_cluster_central_axis(cluster.begin(), cluster.end(), uPCw);
                }

                // Find the angular radius of each ball candidate
                for (BallCandidate& ball : ball_candidates) {
                    // Each candidate corresponds to a cluster currently
                    const std::vector<int>& cluster = clusters[ball.cluster_indices[0]];
                    ball.radius = find_cluster_angular_radius(cluster.begin(), cluster.end(), uPCw, ball.uBCw);
                }

                // Create a adjacency matrix of potentially valid cluster merges
                std::vector<std::vector<size_t>> adj_matrix(ball_candidates.size());
                for (size_t i = 0; i < ball_candidates.size(); ++i) {
                    // Convert cluster radius stored as cos(theta) to angles
                    const double r_i_angle       = std::acos(ball_candidates[i].radius);
                    const Eigen::Vector3d uBCw_i = ball_candidates[i].uBCw;
                    for (size_t j = i + 1; j < ball_candidates.size(); ++j) {
                        const double r_j_angle       = std::acos(ball_candidates[j].radius);
                        const Eigen::Vector3d uBCw_j = ball_candidates[j].uBCw;

                        // If angular separation of 2 cluster axes < sum of their angular radii, from the camera's view
                        // they overlap and can be merged. In use, sum of radii is multiplied by merge_buffer_scalar
                        // and this new combined radius is used for checking the overlap. A merge_buffer_scalar of 1.2
                        // allows merging of clusters with a gap of < 0.2 * sum of radii between their original circles
                        double angular_offset   = std::acos(uBCw_i.dot(uBCw_j));
                        double offset_allowance = (r_i_angle + r_j_angle) * (cfg.merge_buffer_scalar);

                        if (angular_offset < offset_allowance) {
                            adj_matrix[i].push_back(j);
                            adj_matrix[j].push_back(i);
                        }
                    }
                }

                // Find connected components (potential merges) through depth first search of the ball candidates
                std::vector<bool> visited(ball_candidates.size(), false);

                size_t num_candidates = ball_candidates.size();
                for (size_t i = 0; i < num_candidates; ++i) {
                    if (visited[i]) {
                        continue;
                    }

                    std::vector<size_t> stack{};
                    std::vector<size_t> proposed_merge_indices{};

                    visited[i] = true;
                    stack.push_back(i);

                    while (!stack.empty()) {
                        size_t current = stack.back();
                        stack.pop_back();

                        proposed_merge_indices.push_back(current);

                        for (size_t neighbour : adj_matrix[current]) {
                            if (!visited[neighbour]) {
                                stack.push_back(neighbour);
                                visited[neighbour] = true;
                            }
                        }
                    }

                    // End early if there aren't multiple clusters to merge
                    if (proposed_merge_indices.size() <= 1) {
                        continue;
                    }

                    // This is a possible merge
                    BallCandidate ball{};
                    ball.cluster_indices = std::move(proposed_merge_indices);
                    // Combine views of the balls clusters to make a flattened view of all its uPCw mesh indices. Use
                    // its iterators to traverse over all mesh points on the ball simply.
                    auto ball_points = ball.cluster_indices
                                       | std::views::transform([&](size_t i) { return std::views::all(clusters[i]); })
                                       | std::views::join;

                    // Look at the combination of balls with the closest fit to a circle to avoid merging
                    // a ball with a nearby false detection. Greedily look at only the cases nCn and
                    // nCn-1 to minimise checked cases as multiple nearby false detections are unlikely.

                    // Adds up unit vectors of each point in the cluster to find a vector with the
                    // direction of camera to centre of the merged cluster
                    Eigen::Vector3d rBCw = Eigen::Vector3d::Zero();
                    for (size_t idx : ball_points) {
                        rBCw += uPCw.col(idx);
                    }
                    ball.uBCw   = rBCw.normalized();
                    ball.radius = find_cluster_angular_radius(ball_points.begin(), ball_points.end(), uPCw, ball.uBCw);
                    ball.circularity = find_cluster_circularity(ball_points.begin(),
                                                                ball_points.end(),
                                                                neighbours,
                                                                ball.uBCw,
                                                                ball.radius,
                                                                uPCw);
                    // Best removal to maximise circularity
                    std::optional<size_t> best_removed_index{std::nullopt};
                    // Highest entry, defaulting to nCn scaled with merge preference
                    double top_circularity{ball.circularity.value() * cfg.merge_preference};

                    // Cache radius and uBCw associated with the circle with best circularity
                    double associated_radius        = 0.0;
                    Eigen::Vector3d associated_uBCw = Eigen::Vector3d::Zero();

                    // Iterate through the nCn-1 options, removed index is just the unused index from ball indices
                    for (size_t removed_i = 0; removed_i < ball.cluster_indices.size(); ++removed_i) {
                        // Creates a flattened view of mesh indexes
                        std::vector<size_t> filtered_indices = ball.cluster_indices;
                        filtered_indices.erase(filtered_indices.begin() + removed_i);
                        auto proposed_ball =
                            filtered_indices
                            | std::views::transform([&](size_t i) { return std::views::all(clusters[i]); })
                            | std::views::join;

                        // The central axis of the clusters can be found through copying rBCw, then removing all of
                        // the vectors the skipped cluster would have added to it
                        Eigen::Vector3d uBCw = rBCw;
                        for (const int idx : clusters[ball.cluster_indices[removed_i]]) {
                            uBCw -= uPCw.col(idx);
                        }
                        uBCw.normalize();
                        double radius =
                            find_cluster_angular_radius(proposed_ball.begin(), proposed_ball.end(), uPCw, uBCw);

                        double circularity = find_cluster_circularity(proposed_ball.begin(),
                                                                      proposed_ball.end(),
                                                                      neighbours,
                                                                      uBCw,
                                                                      radius,
                                                                      uPCw);

                        if (circularity > top_circularity) {
                            best_removed_index = removed_i;
                            top_circularity    = circularity;
                            associated_uBCw    = uBCw;
                            associated_radius  = radius;
                        }
                    }

                    // Remove the index and update the ball if its going to give a better spherical fit
                    if (best_removed_index.has_value()) {
                        ball.cluster_indices.erase(ball.cluster_indices.begin() + best_removed_index.value());
                        ball.uBCw        = associated_uBCw;
                        ball.radius      = associated_radius;
                        ball.circularity = top_circularity;
                        // Skip merging if its better to do no merges
                        if (ball.cluster_indices.size() <= 1) {
                            log<DEBUG>("No merging done to ball candidate due to worse circularity");
                            continue;
                        }

                        log<DEBUG>("Merged ball created and one possible cluster was removed when making it");
                    }

                    // Note the ball candidates used in merging
                    for (size_t index : ball.cluster_indices) {
                        // Each cluster corresponds to a ball candidate, e.g. index n cluster will have a index n ball
                        // candidate where candidate uses that index as a cluster_index
                        ball_candidates[index].used_in_merge = true;
                    }

                    // The ball as been made and is now used in merging
                    ball_candidates.push_back(std::move(ball));
                }

                log<DEBUG>(fmt::format(
                    "Created {} additional ball candidates through merging {} clusters",
                    ball_candidates.size() - clusters.size(),
                    std::count_if(ball_candidates.begin(), ball_candidates.end(), [](const BallCandidate& candidate) {
                        return candidate.used_in_merge;
                    })));

                // Get a mask for whether a cluster is inside or intersecting the green horizon
                std::vector<bool> accepted_cluster_mask =
                    utility::vision::visualmesh::get_green_horizon_side_mask(clusters,
                                                                             horizon.horizon,
                                                                             rPWw,
                                                                             false,
                                                                             true,
                                                                             true);

                log<DEBUG>(fmt::format("Found {} clusters below green horizon",
                                       std::count(accepted_cluster_mask.begin(), accepted_cluster_mask.end(), true)));

                // Throw out ball candidates where all clusters making it are above the green horizon
                auto cutoff = std::partition(ball_candidates.begin(),
                                             ball_candidates.end(),
                                             [&accepted_cluster_mask](const BallCandidate& ball_candidate) {
                                                 bool kept{false};
                                                 for (const size_t cluster_idx : ball_candidate.cluster_indices) {
                                                     if (accepted_cluster_mask[cluster_idx]) {
                                                         kept = true;
                                                         break;
                                                     }
                                                 }
                                                 return kept;
                                             });

                ball_candidates.resize(std::distance(ball_candidates.begin(), cutoff));

                // Create the Balls message, which will contain a Ball for every cluster that is a valid ball
                auto balls = std::make_unique<Balls>();

                // Balls is still emitted even if there are no balls, to indicate to other modules that no balls are
                // currently visible
                if (ball_candidates.empty()) {
                    log<DEBUG>("Found no balls.");
                    emit(std::move(balls));
                    return;
                }

                // Reserve the memory in advance for efficiency
                balls->balls.reserve(ball_candidates.size());

                balls->id        = horizon.id;         // camera id
                balls->timestamp = horizon.timestamp;  // time when the image was taken
                balls->Hcw       = horizon.Hcw;        // world to camera transform at the time the image was taken

                // World to camera transform, to be used in for loop below
                const Eigen::Isometry3d Hcw(horizon.Hcw.cast<double>());

                // Add ball for each valid ball candidate
                for (BallCandidate& candidate : ball_candidates) {
                    Ball b;

                    // Find central axis of ball, using precomputed measurements from merge logic
                    // uBCw: unit vector from camera to ball central axis in world space
                    // The vectors are in world space, multiply by Rcw to get the central axis in camera space
                    b.uBCc = Hcw.rotation() * candidate.uBCw;

                    // Angular radius, or cos(theta) where theta is the angle between central axis and ball edge
                    b.radius = candidate.radius;

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
                    const double angular_distance = field.ball_radius / std::sqrt(1.0 - b.radius * b.radius);
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
                    const double projection_distance =
                        (field.ball_radius - Hcw.inverse().translation().z()) / candidate.uBCw.z();
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

                    // Discard if cluster was used for merging
                    if (candidate.used_in_merge) {
                        log<DEBUG>("Ball candidate discarded: merged with another detection to create another ball");
                        log<DEBUG>("--------------------------------------------------");
                        // Clusters that were used for merging will show up as orange in NUsight
                        b.colour = keep ? message::conversion::math::vec4(1.0, 0.65, 0.0, 1.0) : b.colour;
                        keep     = false;
                    }

                    // DISCARD IF STANDARD DEVIATION OF ANGLES IS TOO LARGE - CALCULATE DEGREE OF FIT TO CIRCLE
                    // Degree of fit defined as the standard deviation of angle between every rays on the
                    // cluster / and the cone axis (uBCw). If the standard deviation exceeds a given threshold then
                    // it is a bad fit
                    std::vector<double> angles;
                    const size_t num_angles = std::accumulate(candidate.cluster_indices.begin(),
                                                              candidate.cluster_indices.end(),
                                                              0,
                                                              [&clusters](size_t count, size_t cluster_index) {
                                                                  return count + clusters[cluster_index].size();
                                                              });

                    double mean             = 0.0;
                    const double max_radius = std::acos(b.radius);  // largest angle between vectors
                    // Get mean of all the angles in the cluster to then find the standard deviation
                    for (const size_t cluster_idx : candidate.cluster_indices) {
                        for (const int idx : clusters[cluster_idx]) {
                            const double angle = std::acos(candidate.uBCw.dot(uPCw.col(idx))) / max_radius;
                            angles.emplace_back(angle);
                            mean += angle;
                        }
                    }

                    mean /= num_angles;
                    // Calculate standard deviation of angles in cluster
                    double deviation = 0.0;
                    for (const auto& angle : angles) {
                        deviation += (mean - angle) * (mean - angle);
                    }
                    deviation = std::sqrt(deviation / (num_angles - 1));

                    // Check if standard deviation is low enough
                    if (deviation > cfg.maximum_deviation) {

                        log<DEBUG>(fmt::format("Ball discarded: deviation ({}) > maximum_deviation ({})",
                                               deviation,
                                               cfg.maximum_deviation));
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
                    size_t num_clusters_used{candidate.cluster_indices.size()};
                    if (num_clusters_used > 1) {
                        log<DEBUG>(fmt::format("Created through merging {} clusters", num_clusters_used));
                    }
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

                        Eigen::Vector3d ball_position_projection = candidate.uBCw * projection_distance;
                        Eigen::Vector3d ball_position_angular    = candidate.uBCw * angular_distance;
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
