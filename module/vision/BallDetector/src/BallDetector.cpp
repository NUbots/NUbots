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
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <numeric>

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

    BallDetector::BallDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("BallDetector.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.confidence_threshold  = config["confidence_threshold"].as<double>();
            cfg.cluster_points        = config["cluster_points"].as<int>();
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

                log<NUClear::DEBUG>(fmt::format("Partitioned {} points", indices.size()));

                // Cluster all points into ball candidates
                // Points are clustered based on their connectivity to other ball points
                // Clustering is down in two steps
                // 1) Take the set of ball points found above and partition them into potential clusters by
                //    a) Add the first point and its ball neighbours to a cluster
                //    b) Find all other ball points who are neighbours of the points in the cluster
                //    c) Partition all of the indices that are in the cluster
                //    d) Repeat a-c for all points that were not partitioned
                //    e) Delete all partitions smaller than a given threshold
                // 2) Discard all clusters are entirely above the green horizon
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
                                                                                            true);
                clusters.resize(std::distance(clusters.begin(), green_boundary));

                log<NUClear::DEBUG>(fmt::format("Found {} clusters below green horizon", clusters.size()));

                // Create the Balls message, which will contain a Ball for every cluster that is a valid ball
                auto balls = std::make_unique<Balls>();

                // Balls is still emitted even if there are no balls, to indicate to other modules that no balls are
                // currently visible
                if (clusters.empty()) {
                    log<NUClear::DEBUG>("Found no balls.");
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
                for (auto& cluster : clusters) {
                    Ball b;

                    // FIND CENTRAL AXIS OF BALL
                    // Add up all the unit vectors of each point (camera to point in world space) in the cluster to find
                    // an average vector, which represents the central cone axis
                    // uBCw: unit vector from camera to ball central axis in world space
                    Eigen::Vector3d uBCw = Eigen::Vector3d::Zero();
                    for (const auto& idx : cluster) {
                        uBCw += uPCw.col(idx);
                    }
                    uBCw.normalize();  // get cone axis as a unit vector

                    // FIND ANGULAR RADIUS OF BALL
                    // Find the ray (uPCw) with the greatest distance from the central axis (uBCw) to then determine the
                    // largest angular radius possible from the edge points available. Equal to cos(theta), where theta
                    // is the angle between the central ball axis (uBCw) and the edge of the ball. This helps to find
                    // the approximate distance to the ball.
                    double radius = 1.0;
                    for (const auto& idx : cluster) {
                        // Unit vector from the camera to the ball edge, in world space
                        const Eigen::Vector3d& uECw(uPCw.col(idx));
                        // Find the vector that gives the largest angle between the central axis and ball edge
                        // Radius is cos(theta), where theta is the angle, so a smaller radius gives a larger angle.
                        radius = uBCw.dot(uECw) < radius ? uBCw.dot(uECw) : radius;
                    }

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
                    log<NUClear::DEBUG>("**************************************************");
                    log<NUClear::DEBUG>("*                    THROWOUTS                   *");
                    log<NUClear::DEBUG>("**************************************************");
                    bool keep = true;
                    b.colour.fill(1.0);  // a valid ball has a white colour in NUsight

                    // DISCARD IF STANDARD DEVIATION OF ANGLES IS TOO LARGE - CALCULATE DEGREE OF FIT TO CIRCLE
                    // Degree of fit defined as the standard deviation of angle between every rays on the
                    // cluster / and the cone axis (uBCw). If the standard deviation exceeds a given threshold then
                    // it is a bad fit
                    std::vector<double> angles;
                    double mean             = 0.0;
                    const double max_radius = std::acos(radius);  // largest angle between vectors
                    // Get mean of all the angles in the cluster to then find the standard deviation
                    for (const auto& idx : cluster) {
                        const double angle = std::acos(uBCw.dot(uPCw.col(idx))) / max_radius;
                        angles.emplace_back(angle);
                        mean += angle;
                    }
                    mean /= angles.size();
                    // Calculate standard deviation of angles in cluster
                    double deviation = 0.0;
                    for (const auto& angle : angles) {
                        deviation += (mean - angle) * (mean - angle);
                    }
                    deviation = std::sqrt(deviation / (angles.size() - 1));

                    // Check if standard deviation is low enough
                    if (deviation > cfg.maximum_deviation) {

                        log<NUClear::DEBUG>(fmt::format("Ball discarded: deviation ({}) > maximum_deviation ({})",
                                                        deviation,
                                                        cfg.maximum_deviation));
                        log<NUClear::DEBUG>("--------------------------------------------------");
                        // Balls that violate degree of fit will show as green in NUsight
                        b.colour = keep ? message::conversion::math::vec4(0.0, 1.0, 0.0, 1.0) : b.colour;
                        keep     = false;
                    }

                    // DISCARD IF PROJECTION_DISTANCE AND ANGULAR_DISTANCE ARE TOO FAR APART
                    const double max_distance = std::max(projection_distance, angular_distance);

                    if ((std::abs(projection_distance - angular_distance) / max_distance) > cfg.distance_disagreement) {

                        log<NUClear::DEBUG>(
                            fmt::format("Ball discarded: Width and proj distance disagree too much: width = "
                                        "{}, proj = {}",
                                        angular_distance,
                                        projection_distance));
                        log<NUClear::DEBUG>("--------------------------------------------------");
                        // Balls that violate this but not previous checks will show as blue in NUsight
                        b.colour = keep ? message::conversion::math::vec4(0.0, 0.0, 1.0, 1.0) : b.colour;
                        keep     = false;
                    }

                    // DISCARD IF THE DISTANCE FROM THE BALL TO THE ROBOT IS TOO CLOSE
                    // Prevents the robot itself being classed as a ball, ie its arms/hands
                    if (angular_distance < cfg.minimum_ball_distance) {

                        log<NUClear::DEBUG>(fmt::format("Ball discarded: distance ({}) < minimum_ball_distance ({})",
                                                        angular_distance,
                                                        cfg.minimum_ball_distance));
                        log<NUClear::DEBUG>("--------------------------------------------------");
                        // Balls that violate this but not previous checks will show as red in NUsight
                        b.colour = keep ? message::conversion::math::vec4(1.0, 0.0, 0.0, 1.0) : b.colour;
                        keep     = false;
                    }

                    // DISCARD IF THE BALL IS FURTHER THAN THE LENGTH OF THE FIELD
                    if (angular_distance > field.dimensions.field_length) {

                        log<NUClear::DEBUG>(
                            fmt::format("Ball discarded: Distance to ball greater than field length: distance = "
                                        "{}, field length = {}",
                                        angular_distance,
                                        field.dimensions.field_length));
                        log<NUClear::DEBUG>("--------------------------------------------------");
                        // Balls that violate this but not previous checks will show as yellow in NUsight
                        b.colour = keep ? message::conversion::math::vec4(1.0, 0.0, 1.0, 1.0) : b.colour;
                        keep     = false;
                    }

                    log<NUClear::DEBUG>(fmt::format("Camera {}", balls->id));
                    log<NUClear::DEBUG>(fmt::format("radius {}", b.radius));
                    log<NUClear::DEBUG>(fmt::format("Axis {}", b.uBCc.transpose()));
                    log<NUClear::DEBUG>(
                        fmt::format("Distance {} - rBCc {}", angular_distance, b.measurements[0].rBCc.transpose()));
                    log<NUClear::DEBUG>(fmt::format("Projection Distance {} - rBCc",
                                                    projection_distance,
                                                    b.measurements[1].rBCc.transpose()));
                    log<NUClear::DEBUG>(fmt::format("Distance Throwout {}",
                                                    std::abs(projection_distance - angular_distance) / max_distance));
                    log<NUClear::DEBUG>("**************************************************");

                    if (!keep) {
                        b.measurements.clear();
                    }
                    // If the ball passed the checks, add it to the Balls message to be emitted
                    // If it didn't pass the checks, but we're debugging, then emit the ball to see throwouts in NUsight
                    if (keep || log_level <= NUClear::DEBUG) {
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
