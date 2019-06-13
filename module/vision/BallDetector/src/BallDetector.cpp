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

#include "message/input/CameraParameters.h"
#include "message/support/FieldDescription.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/LookUpTable.h"
#include "message/vision/VisionObjects.h"
#include "message/vision/VisualMesh.h"

#include "utility/math/comparison.h"
#include "utility/math/coordinates.h"
#include "utility/math/geometry/Line.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/ransac/Ransac.h"
#include "utility/math/ransac/RansacConeModel.h"
#include "utility/math/ransac/RansacVisualMeshModel.h"
#include "utility/math/vision.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/ClassifiedImage.h"
#include "utility/vision/Vision.h"

#include "utility/math/geometry/Cone.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::input::CameraParameters;

    using message::input::Image;
    using message::support::FieldDescription;
    using message::vision::Ball;
    using message::vision::ClassifiedImage;
    using message::vision::LookUpTable;
    using message::vision::VisualMesh;

    using Plane = utility::math::geometry::Plane<3>;

    using ServoID = utility::input::ServoID;
    using utility::math::geometry::Circle;
    using utility::math::geometry::Cone;
    using utility::math::geometry::Line;
    using utility::math::matrix::Transform3D;
    using utility::math::vision::getCamFromImage;
    using utility::math::vision::getCamFromScreen;
    using utility::math::vision::getImageFromCam;
    using utility::math::vision::getParallaxAngle;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::vision::projectCamToPlane;
    using utility::math::vision::screenToImage;
    using utility::math::vision::screenToImageCts;
    using utility::math::vision::widthBasedDistanceToCircle;

    using utility::math::coordinates::cartesianToSpherical;
    using utility::nusight::graph;

    using utility::math::ransac::Ransac;
    using utility::math::ransac::RansacConeModel;
    using utility::math::ransac::RansacVisualMeshModel;
    using utility::nusight::drawVisionLines;
    using utility::support::Expression;

    using FOURCC = utility::vision::FOURCC;
    using Colour = utility::vision::Colour;

    std::vector<std::vector<arma::vec4>> BallDetector::findClusters(const VisualMesh& mesh,
                                                                    const CameraParameters& cam) {
        // Alias visual mesh parameters
        int dim = mesh.classifications.back().dimensions;

        // Create container for our clusters
        std::vector<std::vector<arma::vec4>> clusters;

        // Create container to mark off used points
        std::set<int> visited_indices;

        // Loop through each coordinate
        for (auto i = 1; i < int(mesh.coordinates.size()) && visited_indices.size() < mesh.coordinates.size(); ++i) {
            // Check if our current coordinate is above threshold
            if (mesh.classifications.back().values[i * dim] >= mesh_seed_confidence_threshold) {
                // Check if our current seed point has already been visited
                if (visited_indices.empty() or visited_indices.find(i) == visited_indices.end()) {
                    std::vector<arma::vec4> cluster;

                    if (print_mesh_debug) {
                        log("New cluster seeded at:", i);
                    }

                    arma::ivec2 seed_coord = convert<int, 2>(mesh.coordinates[i - 1]);
                    // Transform our point into cam space
                    auto seed_cam = getCamFromImage(seed_coord, cam);

                    // Add our seed point for current cluster with confidence
                    cluster.push_back(arma::vec4(
                        {seed_cam[0], seed_cam[1], seed_cam[2], double(mesh.classifications.back().values[i * dim])}));

                    // Add seed point to the set of visited points
                    visited_indices.insert(i);

                    // Create queue for BFS
                    std::queue<int> search_queue;

                    // Add our first seed point onto end of queue
                    search_queue.push(i);

                    int curr_index;

                    while (!search_queue.empty()) {
                        // Set current index and remove from queue as we branch out from seed point
                        curr_index = search_queue.front();
                        search_queue.pop();

                        if (print_mesh_debug) {
                            log("\tSeed        :", curr_index);
                            log("\tPts visited :", visited_indices.size());
                            log("\tSearch items:", search_queue.size());
                        }

                        // Populate the neighbours
                        arma::ivec6 n = convert<int, 6>(mesh.neighbourhood[curr_index]);

                        // Loop through current index and add neighbours onto queue if they are above confidence
                        // threshold
                        for (auto j = 0; j < 6; ++j) {
                            if (print_mesh_debug) {
                                log("\t\tNeighbour : ", n[j]);
                                log("\t\t\tConfidence: ", mesh.classifications.back().values[n[j] * dim]);
                            }

                            // Make sure our confidence is above the threshold
                            // Make sure we haven't visited the point before
                            if ((mesh.classifications.back().values[n[j] * dim] >= mesh_branch_confidence_threshold)
                                && (n[j] != int(mesh.coordinates.size())) && (n[j] != 0)) {
                                if (visited_indices.find(n[j]) == visited_indices.end()) {
                                    search_queue.push(n[j]);  // Add to our BFS queue
                                }

                                // Try to determine if we are on an edge point
                                bool edge           = true;
                                arma::ivec6 local_n = convert<int, 6>(mesh.neighbourhood[n[j]]);
                                for (const auto& l : local_n) {
                                    // Don't TODO if the point is in our list
                                    if ((visited_indices.find(l) == visited_indices.end())
                                        && (mesh.classifications.back().values[l * dim]
                                            >= mesh_branch_confidence_threshold)
                                        && (l != int(mesh.coordinates.size())) && (l != 0)) {
                                        edge = false;
                                    }
                                }

                                if (edge) {
                                    arma::ivec2 point_coord = convert<int, 2>(mesh.coordinates[n[j] - 1]);

                                    auto point_cam = getCamFromImage(point_coord, cam);
                                    cluster.push_back(arma::vec4(
                                        {point_cam[0],
                                         point_cam[1],
                                         point_cam[2],
                                         mesh.classifications.back().values[n[j] * dim]}));  // Add to our cluster
                                }
                            }
                            visited_indices.insert(n[j]);  // Add to our list of visited points
                        }
                    }
                    clusters.push_back(cluster);
                }
            }
        }

        return clusters;
    }

    BallDetector::BallDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , MINIMUM_POINTS_FOR_CONSENSUS(0)
        , MAXIMUM_ITERATIONS_PER_FITTING(0)
        , MAXIMUM_FITTED_MODELS(0)
        , CONSENSUS_ERROR_THRESHOLD(0.0)
        , MAXIMUM_DISAGREEMENT_RATIO(0.0)
        , maximum_relative_seed_point_distance(0.0)
        , measurement_distance_variance_factor(0.0)
        , measurement_bearing_variance(0.0)
        , measurement_elevation_variance(0.0)
        , green_ratio_threshold(0.0)
        , green_radial_samples(0.0)
        , green_angular_samples(0.0)
        , kmeansClusterer()
        , lastFrame()
        , print_throwout_logs(false) {


        on<Configuration>("BallDetector.yaml").then([this](const Configuration& config) {
            MINIMUM_POINTS_FOR_CONSENSUS = config["ransac"]["minimum_points_for_consensus"].as<uint>();
            CONSENSUS_ERROR_THRESHOLD    = config["ransac"]["consensus_error_threshold"].as<Expression>();

            MAXIMUM_ITERATIONS_PER_FITTING = config["ransac"]["maximum_iterations_per_fitting"].as<uint>();
            MAXIMUM_FITTED_MODELS          = config["ransac"]["maximum_fitted_models"].as<uint>();
            MAXIMUM_DISAGREEMENT_RATIO     = config["maximum_disagreement_ratio"].as<Expression>();

            mesh_branch_confidence_threshold = config["visual_mesh"]["mesh_branch_confidence_threshold"];
            mesh_seed_confidence_threshold   = config["visual_mesh"]["mesh_seed_confidence_threshold"];


            maximum_relative_seed_point_distance = config["maximum_relative_seed_point_distance"].as<double>();

            measurement_distance_variance_factor = config["measurement_distance_variance_factor"].as<Expression>();
            measurement_bearing_variance         = config["measurement_bearing_variance"].as<Expression>();
            measurement_elevation_variance       = config["measurement_elevation_variance"].as<Expression>();

            green_ratio_threshold = config["green_ratio_threshold"].as<Expression>();
            green_radial_samples  = config["green_radial_samples"].as<Expression>();
            green_angular_samples = config["green_angular_samples"].as<Expression>();

            ball_angular_cov = config["ball_angular_cov"].as<arma::vec>();

            kmeansClusterer.configure(config["clustering"]);

            print_throwout_logs = config["print_throwout_logs"].as<bool>();
            print_mesh_debug    = config["print_mesh_debug"].as<bool>();
            draw_cluster        = config["draw_cluster"].as<bool>();

            lastFrame.time = NUClear::clock::now();
        });

        on<Trigger<VisualMesh>,
           With<FieldDescription>,
           With<CameraParameters>,
           With<ClassifiedImage>,
           With<LookUpTable>>()
            .then(
                "Visual Mesh",
                [this](const VisualMesh& mesh,
                       const FieldDescription& field,
                       const CameraParameters& cam,
                       std::shared_ptr<const ClassifiedImage> rawImage,
                       const LookUpTable& lut) {
                    // We need to gather all points which have a confidence prediction of over MAX_PREDICT_THRESH
                    // Then BFS to all neighbouring points which have a confidence prediction of at least
                    // MIN_PREDICT_THRESH
                    // We then need to create ransac models for each of these 'clusters' to fit a circle
                    const auto& image = *rawImage;

                    // Get our coordinate clusters in camera space
                    std::vector<std::vector<arma::vec4>> clusters = findClusters(mesh, cam);

                    if (print_mesh_debug) {
                        log("Number of clusters found:", clusters.size());
                    }

                    auto balls = std::make_unique<std::vector<Ball>>();
                    if (clusters.size() > 0) {
                        balls->reserve(clusters.size());
                    }

                    for (const auto& cluster : clusters) {
                        Ball b;

                        // Average all the points in the cluster to find the center
                        arma::vec3 center(arma::fill::zeros);
                        double max_x = -1.0, min_x = 1.0;
                        double max_y = -1.0, min_y = 1.0;
                        double max_z = -1.0, min_z = 1.0;
                        for (const auto& point : cluster) {
                            center += point.head(3);
                            min_x = std::min(min_x, point[0]);
                            max_x = std::max(max_x, point[0]);
                            min_y = std::min(min_y, point[1]);
                            max_y = std::max(max_y, point[1]);
                            min_z = std::min(min_z, point[2]);
                            max_z = std::max(max_z, point[2]);
                        }

                        center /= cluster.size();
                        center = arma::normalise(center);

                        // Use the average of the extreme coordinates to determine the radius
                        double radius =
                            (std::abs(max_x - min_x) + std::abs(max_y - min_y) + std::abs(max_z - min_z)) / 6.0;

                        // Work out the width distance
                        arma::vec3 topCam   = arma::normalise(center + arma::vec3({0, 0, radius}));
                        arma::vec3 baseCam  = arma::normalise(center - arma::vec3({0, 0, radius}));
                        arma::vec3 leftCam  = arma::normalise(center + arma::vec3({0, radius, 0}));
                        arma::vec3 rightCam = arma::normalise(center - arma::vec3({0, radius, 0}));

                        arma::vec2 top   = projectCamSpaceToScreen(topCam, cam);
                        arma::vec2 base  = projectCamSpaceToScreen(baseCam, cam);
                        arma::vec2 left  = projectCamSpaceToScreen(leftCam, cam);
                        arma::vec2 right = projectCamSpaceToScreen(rightCam, cam);

                        // https://en.wikipedia.org/wiki/Angular_diameter
                        double delta    = std::acos(arma::dot(topCam, baseCam));
                        double distance = field.ball_radius / std::sin(delta * 0.5);

                        // Work out how far away the ball must be to be at the distance it is from the camera
                        arma::vec3 rBCc = center * distance;

                        // Attach the measurement to the object
                        b.measurements.push_back(Ball::Measurement());
                        b.measurements.back().rBCc       = convert<double, 3, 1>(rBCc);
                        b.measurements.back().covariance = convert<double, 3>(ball_angular_cov).asDiagonal();

                        // Ball cam space info
                        b.cone.axis     = convert<double, 3>(center);
                        b.cone.gradient = -std::numeric_limits<double>::max();

                        // Percentage of green classified points in cluster
                        auto numGreen    = 0;
                        float greenRatio = 0.;

                        // Cast raw image from classified image
                        auto pixelImage = const_cast<Image*>(image.image.get())->shared_from_this();

                        for (const auto& point : cluster) {
                            // Calculate image space pixel coordinate of point
                            auto pixel = screenToImage(projectCamSpaceToScreen(point.head(3), cam),
                                                       convert<uint, 2>((*pixelImage).dimensions));

                            // Check our cluster pointer for the maximum gradient
                            b.cone.gradient = std::tan(std::acos(arma::dot(center, point.head(3))));

                            // Add our points
                            b.edgePoints.push_back(convert<double, 3>(point.head(3)));

                            // Calculate number of classified green points in cluster
                            char c = static_cast<char>(
                                utility::vision::getPixelColour(lut,
                                                                getPixel(pixel[0],
                                                                         pixel[1],
                                                                         (*pixelImage).dimensions[0],
                                                                         (*pixelImage).dimensions[1],
                                                                         (*pixelImage).data,
                                                                         static_cast<FOURCC>((*pixelImage).format))));

                            if (c == Colour::GREEN) {
                                numGreen++;
                            }
                        }

                        if (cluster.size() > 0) {
                            greenRatio = numGreen / float(cluster.size());
                        }


                        // Angular positions from the camera
                        b.visObject.screenAngular = convert<double, 2>(cartesianToSpherical(center).rows(1, 2));
                        b.visObject.angularSize << getParallaxAngle(left, right, cam), getParallaxAngle(top, base, cam);
                        b.visObject.classifiedImage = const_cast<ClassifiedImage*>(rawImage.get())->shared_from_this();
                        b.visObject.timestamp       = NUClear::clock::now();
                        b.visObject.sensors         = image.sensors;

                        if (print_mesh_debug) {
                            log("Gradient",
                                b.cone.gradient,
                                "Center",
                                center.t(),
                                "Radius",
                                radius,
                                "Distance",
                                distance,
                                "rBCc",
                                rBCc.t(),
                                "Vision object: screenAngular",
                                b.visObject.screenAngular.transpose(),
                                "Vision object: angularSize",
                                b.visObject.angularSize.transpose());
                        }


                        /***********************************************
                         *                  THROWOUTS                  *
                         ***********************************************/

                        // CENTRE OF BALL IS ABOVE THE VISUAL HORIZON
                        arma::ivec2 centre_im = getImageFromCam(center, cam);
                        if (utility::vision::visualHorizonAtPoint(image, centre_im[0]) > centre_im[1]
                            || arma::dot(convert<double, 3>(image.horizon_normal), center) > 0) {
                            if (print_throwout_logs) {
                                log("Ball discarded: arma::dot(image.horizon_normal,ballCentreRay) > 0 ");
                                log("Horizon normal = ", image.horizon_normal.transpose());
                                log("Ball centre ray = ", center.t());
                            }
                            continue;
                        }

                        // DISTANCE IS LESS THAN HALF OF CAM HEIGHT
                        const auto& sensors            = *image.sensors;
                        const arma::mat44& camToGround = convert<double, 4, 4>(sensors.camToGround);
                        const double cameraHeight      = camToGround(2, 3);

                        if (distance < cameraHeight * 0.5) {
                            if (print_throwout_logs) {
                                log("Ball discarded: distance < cameraHeight * 0.5");
                                log("distance =", distance, "cameraHeight =", cameraHeight);
                            }
                            continue;
                        }

                        // IF THE DISAGREEMENT BETWEEN THE ANGULAR AND PROJECTION BASED DISTANCES ARE TOO LARGE
                        // Project this vector to a plane midway through the ball
                        Plane ballBisectorPlane({0, 0, 1}, {0, 0, field.ball_radius});
                        arma::vec3 ballCentreGroundProj     = projectCamToPlane(center, camToGround, ballBisectorPlane);
                        double ballCentreGroundProjDistance = arma::norm(ballCentreGroundProj);

                        if (std::abs((distance - ballCentreGroundProjDistance)
                                     / std::max(ballCentreGroundProjDistance, distance))
                            > MAXIMUM_DISAGREEMENT_RATIO) {
                            if (print_throwout_logs)
                                log("Ball discarded: Width and proj distance disagree too much: width =",
                                    distance,
                                    "proj =",
                                    ballCentreGroundProjDistance);
                            continue;
                        }

                        // IF THE BALL IS FURTHER THAN THE LENGTH OF THE FIELD
                        if (distance > field.dimensions.field_length) {
                            if (print_throwout_logs) {
                                log("Ball discarded: Distance to ball greater than field length: distance =",
                                    distance,
                                    "field length=",
                                    field.dimensions.field_length);
                            }
                            continue;
                        }

                        // IF THE BALL IS HAS TOO HIGH OF A GREEN RATIO
                        if (greenRatio > green_ratio_threshold) {
                            if (print_throwout_logs) {
                                log("Ball discarded: Green ratio (",
                                    greenRatio,
                                    ") > green ratio threshold (",
                                    green_ratio_threshold,
                                    ")");
                                continue;
                            }
                        }

                        balls->push_back(std::move(b));
                    }

                    /***********************************************
                     *                  CLUSTERS DRAW              *
                     ***********************************************/

                    if (draw_cluster) {
                        std::vector<
                            std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>,
                            Eigen::aligned_allocator<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>>>
                            lines;
                        for (size_t i = 0; i < clusters.size(); ++i) {

                            arma::vec3 axis(arma::fill::zeros);
                            for (const auto& point : clusters[i]) {
                                axis += point.head(3);
                            }

                            axis /= clusters[i].size();
                            axis = arma::normalise(axis);

                            Eigen::Vector2i center = convert<int, 2>(screenToImage(
                                projectCamSpaceToScreen(axis, cam), convert<uint, 2>(cam.imageSizePixels)));

                            // Average all the points in the cluster to find the center
                            for (size_t j = 0; j < (clusters[i].size()); ++j) {
                                Eigen::Vector4d colour(clusters[i][j][3] >= 0.5, 0.50, clusters[i][j][3] < 0.5, 1);

                                Eigen::Vector2i point =
                                    convert<int, 2>(screenToImage(projectCamSpaceToScreen(clusters[i][j].head(3), cam),
                                                                  convert<uint, 2>(cam.imageSizePixels)));

                                lines.emplace_back(center.cast<int>(), point.cast<int>(), colour);
                            }
                        }
                        emit(utility::nusight::drawVisionLines(lines));
                    }
                    emit(std::move(balls));
                });
    }
}  // namespace vision
}  // namespace module
