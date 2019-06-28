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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "BallDetector.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "extension/Configuration.h"

#include "message/support/FieldDescription.h"
#include "message/vision/Ball.h"
#include "message/vision/GreenHorizon.h"

#include "utility/math/coordinates.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/visualmesh/VisualMesh.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::support::FieldDescription;
    using message::vision::Ball;
    using message::vision::Balls;
    using message::vision::GreenHorizon;

    using utility::math::coordinates::cartesianToSpherical;

    static constexpr int BALL_INDEX = 0;

    BallDetector::BallDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("BallDetector.yaml").then([this](const Configuration& cfg) {
            config.confidence_threshold  = cfg["confidence_threshold"].as<float>();
            config.cluster_points        = cfg["cluster_points"].as<int>();
            config.minimum_ball_distance = cfg["minimum_ball_distance"].as<float>();
            config.distance_disagreement = cfg["distance_disagreement"].as<float>();
            config.maximum_deviation     = cfg["maximum_deviation"].as<float>();
            config.ball_angular_cov      = convert(cfg["ball_angular_cov"].as<arma::vec>()).cast<float>().asDiagonal();
            config.debug                 = cfg["debug"].as<bool>();
        });

        on<Trigger<GreenHorizon>, With<FieldDescription>, Buffer<2>>().then(
            "Visual Mesh", [this](const GreenHorizon& horizon, const FieldDescription& field) {
                // Convenience variables
                const auto& cls                                     = horizon.mesh->classifications;
                const auto& neighbours                              = horizon.mesh->neighbourhood;
                const Eigen::Matrix<float, 3, Eigen::Dynamic>& rays = horizon.mesh->rays;
                const float world_offset                            = std::atan2(horizon.Hcw(0, 1), horizon.Hcw(0, 0));

                // Get some indices to partition
                std::vector<int> indices(horizon.mesh->indices.size());
                std::iota(indices.begin(), indices.end(), 0);

                // Partition the indices such that we only have the ball points that dont have ball surrounding them
                auto boundary = utility::vision::visualmesh::partition_points(
                    indices.begin(), indices.end(), neighbours, [&](const int& idx) {
                        return idx == indices.size() || (cls(BALL_INDEX, idx) >= config.confidence_threshold);
                    });

                // Discard indices that are not on the boundary and are not below the green horizon
                indices.resize(std::distance(indices.begin(), boundary));

                // Cluster all points into ball candidates
                // Points are clustered based on their connectivity to other ball points
                // Clustering is down in two steps
                // 1) We take the set of ball points found above and partition them into potential clusters by
                //    a) Add the first point and its ball neighbours to a cluster
                //    b) Find all other ball points who are neighbours of the points in the cluster
                //    c) Partition all of the indices that are in the cluster
                //    d) Repeat a-c for all points that were not partitioned
                //    e) Delete all partitions smaller than a given threshold
                // 2) Discard all clusters are entirely above the green horizon
                std::vector<std::vector<int>> clusters;
                utility::vision::visualmesh::cluster_points(
                    indices.begin(), indices.end(), neighbours, config.cluster_points, clusters);

                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));
                }

                auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(clusters.begin(),
                                                                                            clusters.end(),
                                                                                            horizon.horizon.begin(),
                                                                                            horizon.horizon.end(),
                                                                                            rays,
                                                                                            false,
                                                                                            true);
                clusters.resize(std::distance(clusters.begin(), green_boundary));

                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Found {} clusters below green horizon", clusters.size()));
                }

                if (clusters.size() > 0) {
                    auto balls = std::make_unique<Balls>();
                    balls->balls.reserve(clusters.size());

                    balls->camera_id = horizon.camera_id;
                    balls->timestamp = horizon.timestamp;
                    balls->Hcw       = horizon.Hcw;

                    for (auto& cluster : clusters) {
                        Ball b;

                        // Average the cluster to get the cones axis
                        Eigen::Vector3f axis = Eigen::Vector3f::Zero();
                        for (const auto& idx : cluster) {
                            axis += rays.col(idx);
                        }
                        axis /= cluster.size();
                        axis.normalize();

                        // Find the ray with the greatest distance from the axis
                        // Should we use the average distance instead?
                        float radius = 1.0f;
                        for (const auto& idx : cluster) {
                            const Eigen::Vector3f& ray(rays.col(idx));
                            if (axis.dot(ray) < radius) {
                                radius = axis.dot(ray);
                            }
                        }

                        // Ball cam space info
                        b.cone.axis     = horizon.Hcw.topLeftCorner<3, 3>().cast<float>() * axis;
                        float proj      = 1.0f / radius;
                        b.cone.gradient = std::sqrt(proj * proj - 1.0f);
                        b.cone.radius   = radius;

                        // https://en.wikipedia.org/wiki/Angular_diameter
                        float distance = field.ball_radius / std::sqrt(1.0f - radius * radius);

                        // Attach the measurement to the object (distance from camera to ball)
                        b.measurements.push_back(Ball::Measurement());
                        b.measurements.back().rBCc       = cartesianToSpherical(b.cone.axis * distance);
                        b.measurements.back().covariance = config.ball_angular_cov;

                        // Angular positions from the camera
                        b.screen_angular = cartesianToSpherical(axis).tail<2>();
                        b.angular_size   = Eigen::Vector2f::Constant(std::acos(radius));


                        /***********************************************
                         *                  THROWOUTS                  *
                         ***********************************************/

                        if (config.debug) {
                            log<NUClear::DEBUG>("**************************************************");
                            log<NUClear::DEBUG>("*                    THROWOUTS                   *");
                            log<NUClear::DEBUG>("**************************************************");
                        }
                        bool keep = true;
                        b.colour.fill(1.0f);

                        // CALCULATE DEGREE OF FIT
                        // Degree of fit defined as the standard deviation of angle between every rays on the cluster /
                        // and the cone axis. If the standard deviation exceeds a given threshold then we have a bad
                        // fit
                        std::vector<float> angles;
                        float mean             = 0.0f;
                        const float max_radius = std::acos(radius);
                        for (const auto& idx : cluster) {
                            const float angle = std::acos(axis.dot(rays.col(idx))) / max_radius;
                            angles.emplace_back(angle);
                            mean += angle;
                        }
                        mean /= angles.size();
                        float deviation = 0.0f;
                        for (const auto& angle : angles) {
                            deviation += (mean - angle) * (mean - angle);
                        }
                        deviation = std::sqrt(deviation / (angles.size() - 1));

                        if (deviation > config.maximum_deviation) {
                            if (config.debug) {
                                log<NUClear::DEBUG>(
                                    fmt::format("Ball discarded: deviation ({}) > maximum_deviation ({})",
                                                deviation,
                                                config.maximum_deviation));
                                log<NUClear::DEBUG>("--------------------------------------------------");
                            }
                            b.colour = keep ? message::conversion::math::fvec4(0.0f, 1.0f, 0.0f, 1.0f) : b.colour;
                            keep     = false;
                        }

                        // DISTANCE IS TOO CLOSE
                        if (distance < config.minimum_ball_distance) {
                            if (config.debug) {
                                log<NUClear::DEBUG>(
                                    fmt::format("Ball discarded: distance ({}) < minimum_ball_distance ({})",
                                                distance,
                                                config.minimum_ball_distance));
                                log<NUClear::DEBUG>("--------------------------------------------------");
                            }
                            b.colour = keep ? message::conversion::math::fvec4(1.0f, 0.0f, 0.0f, 1.0f) : b.colour;
                            keep     = false;
                        }

                        // IF THE DISAGREEMENT BETWEEN THE ANGULAR AND PROJECTION BASED DISTANCES ARE TOO LARGE
                        // Intersect cone axis vector with a plane midway through the ball with normal vector (0, 0, 1)
                        // Do this in world space, not camera space!
                        // https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection#Algebraic_form
                        // Plane normal = (0, 0, 1)
                        // Point in plane = (0, 0, field.ball_radius)
                        // Line direction = axis
                        // Point on line = camera = Hcw.topRightCorner<3, 1>()
                        const float d = (field.ball_radius - horizon.Hcw(2, 3)) / axis.z();
                        const Eigen::Vector3f ball_projection =
                            axis * d + horizon.Hcw.topRightCorner<3, 1>().cast<float>();
                        const float projection_distance = ball_projection.norm();
                        const float max_distance        = std::max(projection_distance, distance);

                        if ((std::abs(projection_distance - distance) / max_distance) > config.distance_disagreement) {
                            if (config.debug) {
                                log<NUClear::DEBUG>(
                                    fmt::format("Ball discarded: Width and proj distance disagree too much: width = "
                                                "{}, proj = {}",
                                                distance,
                                                projection_distance));
                                log<NUClear::DEBUG>("--------------------------------------------------");
                            }
                            b.colour = keep ? message::conversion::math::fvec4(0.0f, 0.0f, 1.0f, 1.0f) : b.colour;
                            keep     = false;
                        }

                        // IF THE BALL IS FURTHER THAN THE LENGTH OF THE FIELD
                        if (distance > field.dimensions.field_length) {
                            if (config.debug) {
                                log<NUClear::DEBUG>(fmt::format(
                                    "Ball discarded: Distance to ball greater than field length: distance = "
                                    "{}, field length = {}",
                                    distance,
                                    field.dimensions.field_length));
                                log<NUClear::DEBUG>("--------------------------------------------------");
                            }
                            b.colour = keep ? message::conversion::math::fvec4(1.0f, 0.0f, 1.0f, 1.0f) : b.colour;
                            keep     = false;
                        }

                        if (config.debug) {
                            log<NUClear::DEBUG>(fmt::format("Camera {}", balls->camera_id));
                            log<NUClear::DEBUG>(
                                fmt::format("Gradient {} - cos(theta) {}", b.cone.gradient, b.cone.radius));
                            log<NUClear::DEBUG>(fmt::format("Axis {}", b.cone.axis.transpose()));
                            log<NUClear::DEBUG>(
                                fmt::format("Distance {} - rBCc {}", distance, b.measurements.back().rBCc.transpose()));
                            log<NUClear::DEBUG>(fmt::format("screen_angular {} - angular_size {}",
                                                            b.screen_angular.transpose(),
                                                            b.angular_size.transpose()));
                            log<NUClear::DEBUG>(fmt::format("Projection Distance {}", projection_distance));
                            log<NUClear::DEBUG>(fmt::format("Distance Throwout {}",
                                                            std::abs(projection_distance - distance) / max_distance));
                            log<NUClear::DEBUG>("**************************************************");
                        }

                        if (config.debug || keep) {
                            balls->balls.push_back(std::move(b));
                        }
                    }
                    emit(std::move(balls));
                }
                else {
                    if (config.debug) {
                        log<NUClear::DEBUG>("Found no balls.");
                    }
                }
            });
    }  // namespace vision
}  // namespace vision
}  // namespace module


/*****************************************************************************
 * Cone fitting ... kind of broken
                    // Get axis unit vector from 3 unit vectors
                    auto cone_from_points = [&](const int& a, const int& b, const int& c) {
                        Eigen::Matrix3f A;
                        A.row(0) = rays.row(a);
                        A.row(1) = rays.row(b);
                        A.row(2) = rays.row(c);
                        return A.householderQr().solve(Eigen::Vector3f::Ones()).normalized();
                    };

                    // Determine if point d is contained within the cone made from a, b, and c
                    // Returns which of the points a (0), b (1), c (2), or d (3) is contained in the cone formed by the
                    // other 3 points
                    // Returns -1 if none of the cones formed by any permutation is valid
                    auto point_in_cone = [&](const int& a, const int& b, const int& c, const int& d) {
                        std::array<int, 4> perms = {a, b, c, d};
                        for (int i = 0; i < 4; ++i) {
                            const Eigen::Vector3f& p0 = rays.row(perms[i]);
                            const Eigen::Vector3f& p1 = rays.row(perms[(i + 1) % 4]);
                            const Eigen::Vector3f& p2 = rays.row(perms[(i + 2) % 4]);
                            const Eigen::Vector3f& p3 = rays.row(perms[(i + 3) % 4]);
                            Eigen::Vector3f x = cone_from_points(perms[i], perms[(i + 1) % 4], perms[(i + 2) % 4]);

                            // Cone is valid
                            if (utility::math::almost_equal(p0.dot(x), p1.dot(x), 2)
                                && utility::math::almost_equal(p0.dot(x), p2.dot(x), 2)) {
                                // Point is contained within the cone of a, b, c
                                if (x.dot(p0) > config.maximum_cone_radius) {
                                    if (x.dot(p3) >= p0.dot(x)) {
                                        return 3 - i;
                                    }
                                }
                            }
                        }
                        // Everything is just horribly bad, pick on the new guy
                        return 3;
                    };

                    for (auto& cluster : clusters) {
                        Ball b;

                        // Shuffle to be random
                        std::random_shuffle(cluster.begin(), cluster.end());
                        while (cluster.size() > 3) {
                            // Consider changing this to use only 2 vectors to form the axis for the cone
                            cluster.erase(std::next(cluster.begin(),
                                                    point_in_cone(cluster[0], cluster[1], cluster[2], cluster[3])));
                        }

                        // The first (only) 3 points left in the cluster are used to form the cone
                        Eigen::Vector3f axis = cone_from_points(cluster[0], cluster[1], cluster[2]);
                        double radius        = axis.dot(rays.row(cluster[0]));

                        /// DO MEASUREMENTS AND THROWOUTS
                    }

 *****************************************************************************/
