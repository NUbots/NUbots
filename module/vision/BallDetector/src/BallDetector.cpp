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


#include "extension/Configuration.h"

#include "message/support/FieldDescription.h"
#include "message/vision/Ball.h"
#include "message/vision/GreenHorizon.h"

#include "utility/math/comparison.h"
#include "utility/math/coordinates.h"
#include "utility/math/geometry/ConvexHull.h"
#include "utility/math/geometry/Line.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/vision.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/Vision.h"

#include "utility/math/geometry/Cone.h"

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
            config.maximum_cone_radius   = std::cos(cfg["maximum_cone_radius"].as<float>());
            config.minimum_ball_distance = cfg["minimum_ball_distance"].as<float>();
            config.distance_disagreement = cfg["distance_disagreement"].as<float>();
            config.ball_angular_cov      = convert<double, 3>(cfg["ball_angular_cov"].as<arma::vec>()).asDiagonal();
            config.debug                 = cfg["debug"].as<bool>();
        });

        on<Trigger<GreenHorizon>, With<FieldDescription>, Buffer<2>>().then(
            "Visual Mesh", [this](const GreenHorizon& horizon, const FieldDescription& field) {
                // Convenience variables
                const auto& cls                                     = horizon.mesh->classifications;
                const auto& neighbours                              = horizon.mesh->neighbourhood;
                const Eigen::Matrix<float, Eigen::Dynamic, 3>& rays = horizon.mesh->rays;
                const float world_offset                            = std::atan2(horizon.Hcw(0, 1), horizon.Hcw(0, 0));

                // Get some indices to partition
                std::vector<int> indices(horizon.mesh->indices.size());
                std::iota(indices.begin(), indices.end(), 0);

                // Partition the indices such that we only have the ball points that dont have ball surrounding them
                auto boundary = std::partition(indices.begin(), indices.end(), [&](const int& idx) {
                    // If point is not a field or field line point then we also ignore it
                    if (cls(idx, BALL_INDEX) < config.confidence_threshold) {
                        return false;
                    }
                    // If at least one neighbour is not a ball then this point should be on the edge
                    for (int n = 0; n < 6; ++n) {
                        const int neighbour_idx = neighbours(idx, n);
                        if (neighbour_idx == indices.size()) {
                            continue;
                        }
                        if (cls(neighbour_idx, BALL_INDEX) < config.confidence_threshold) {
                            return true;
                        }
                    }
                    return false;
                });

                // Discard all ball points that are not below the green horizon
                auto horizon_boundary = std::partition(indices.begin(), boundary, [&](const int& idx) {
                    return utility::math::geometry::point_under_hull(
                        rays.row(idx), horizon.horizon.begin(), horizon.horizon.end(), true);
                });

                // Discard indices that are not on the boundary and are not below the green horizon
                indices.resize(std::distance(indices.begin(), horizon_boundary));

                // Sort indices by increasing theta
                utility::math::geometry::sort_by_theta(indices.begin(), indices.end(), rays, world_offset);

                // Cluster all points into ball candidates
                // Points are clustered based on their connectivity to other ball points
                // Clustering is down in three steps
                // 1) We take the set of ball points found above and partition them into potential clusters by
                //    a) Add the first point and its ball neighbours to a cluster
                //    b) Find all other ball points who are neighbours of the points in the cluster
                //    c) Partition all of the indices that are in the cluster
                //    d) Repeat a-c for all points that were not partitioned
                // 2) Discard all partitions smaller than a given threshold
                //    This also copies the indices for partitions larger than the threshold into separate vectors
                auto first                                         = indices.begin();
                std::vector<std::vector<int>::iterator> partitions = {first};

                while (first != indices.end()) {
                    // Add the first point and all of its neighbours to the cluster
                    std::vector<int> cluster = {*first};
                    for (int n = 0; n < 6; ++n) {
                        const int neighbour_idx = neighbours(*first, n);
                        if (neighbour_idx == indices.size()) {
                            continue;
                        }
                        if (std::find(indices.begin(), indices.end(), neighbour_idx) == cluster.end()) {
                            cluster.push_back(neighbour_idx);
                        }
                    }
                    partitions.push_back(std::partition(std::next(first), indices.end(), [&](const int& idx) {
                        // At least one neighbour of the current point must be a neighbour of one of the points in the
                        // current cluster
                        for (const auto& search_idx : cluster) {
                            for (int n = 0; n < 6; ++n) {
                                const int neighbour_idx = neighbours(search_idx, n);
                                // Make sure we haven't already considered this point
                                if (std::find(cluster.begin(), cluster.end(), idx) != cluster.end()) {
                                    return true;
                                }
                                if (neighbour_idx == idx) {
                                    cluster.push_back(idx);
                                    return true;
                                }
                            }
                        }
                        return false;
                    }));

                    first = partitions.back();
                }

                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Found {} partitions", partitions.size()));
                }

                // Discard all partitions smaller than a given size
                // Copy the indices of the large clusters into separate vectors
                std::vector<std::vector<int>> clusters;
                for (int i = 1; i < partitions.size(); ++i) {
                    // We must have a minimum of 3 points to make a ball
                    if (std::distance(partitions[i - 1], partitions[i]) >= 3) {
                        clusters.emplace_back(partitions[i - 1], partitions[i]);
                    }
                }

                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));
                }

                auto balls = std::make_unique<Balls>();
                if (clusters.size() > 0) {
                    balls->balls.reserve(clusters.size());
                }

                balls->timestamp = horizon.timestamp;
                balls->Hcw       = horizon.Hcw;

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
                        Eigen::Vector3f x         = cone_from_points(perms[i], perms[(i + 1) % 4], perms[(i + 2) % 4]);

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
                        cluster.erase(
                            std::next(cluster.begin(), point_in_cone(cluster[0], cluster[1], cluster[2], cluster[3])));
                    }

                    // The first (only) 3 points left in the cluster are used to form the cone
                    Eigen::Vector3f axis = cone_from_points(cluster[0], cluster[1], cluster[2]);
                    double radius        = axis.dot(rays.row(cluster[0]));

                    // Ball cam space info
                    b.cone.axis     = horizon.Hcw.topLeftCorner<3, 3>() * axis.cast<double>();
                    double proj     = 1.0 / radius;
                    b.cone.gradient = std::sqrt(proj * proj - 1.0);

                    // https://en.wikipedia.org/wiki/Angular_diameter
                    double distance = field.ball_radius * M_SQRT2 / std::sqrt(1.0 - radius);

                    // Attach the measurement to the object (distance from camera to ball)
                    b.measurements.push_back(Ball::Measurement());
                    b.measurements.back().rBCc       = b.cone.axis * distance;
                    b.measurements.back().covariance = config.ball_angular_cov;

                    // Angular positions from the camera
                    b.screen_angular = cartesianToSpherical(axis).cast<double>().tail<2>();
                    b.angular_size   = Eigen::Vector2d::Constant(std::acos(radius));

                    if (config.debug) {
                        log("Gradient",
                            b.cone.gradient,
                            "Center",
                            b.cone.axis.transpose(),
                            "Radius",
                            radius,
                            "Distance",
                            distance,
                            "rBCc",
                            b.measurements.back().rBCc.transpose(),
                            "screen_angular",
                            b.screen_angular.transpose(),
                            "angular_size",
                            b.angular_size.transpose());
                    }


                    /***********************************************
                     *                  THROWOUTS                  *
                     ***********************************************/

                    // DISTANCE IS TOO CLOSE
                    if (distance < config.minimum_ball_distance) {
                        if (config.debug) {
                            log(fmt::format("Ball discarded: distance ({}) < minimum_ball_distance ({})",
                                            distance,
                                            config.minimum_ball_distance));
                        }
                        continue;
                    }

                    // IF THE DISAGREEMENT BETWEEN THE ANGULAR AND PROJECTION BASED DISTANCES ARE TOO LARGE
                    // Project this vector to a plane midway through the ball
                    Eigen::Vector3d ball_projection(b.cone.axis * (horizon.Hcw(2, 3) - field.ball_radius)
                                                    / b.cone.axis.z());
                    if (std::abs(ball_projection.norm() - distance) / std::max(ball_projection.norm(), distance)
                        > config.distance_disagreement) {
                        if (config.debug)
                            log(fmt::format(
                                "Ball discarded: Width and proj distance disagree too much: width = {}, proj = {}",
                                distance,
                                ball_projection.norm()));
                        continue;
                    }

                    // IF THE BALL IS FURTHER THAN THE LENGTH OF THE FIELD
                    if (distance > field.dimensions.field_length) {
                        if (config.debug) {
                            log(
                                fmt::format("Ball discarded: Distance to ball greater than field length: distance = "
                                            "{}, field length = {}",
                                            distance,
                                            field.dimensions.field_length));
                        }
                        continue;
                    }

                    balls->balls.push_back(std::move(b));
                }
                emit(std::move(balls));
            });
    }
}  // namespace vision
}  // namespace module
