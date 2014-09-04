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

#include "messages/vision/ClassifiedImage.h"
#include "messages/vision/VisionObjects.h"
#include "messages/input/CameraParameters.h"
#include "messages/support/Configuration.h"
#include "messages/support/FieldDescription.h"

#include "utility/math/geometry/Plane.h"

#include "utility/math/ransac/Ransac.h"
#include "utility/math/ransac/RansacCircleModel.h"
#include "utility/math/vision.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/coordinates.h"

namespace modules {
namespace vision {

    using messages::input::CameraParameters;
    using messages::input::Sensors;

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;
    using messages::vision::VisionObject;
    using messages::vision::Ball;

    using Plane = utility::math::geometry::Plane<3>;

    using utility::math::vision::widthBasedDistanceToCircle;
    using utility::math::vision::projectCamToPlane;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::getCamFromScreen;
    using utility::math::vision::getParallaxAngle;
    using utility::math::vision::projectCamSpaceToScreen;

    using utility::math::coordinates::cartesianToSpherical;
    using utility::nubugger::graph;

    using messages::support::Configuration;
    using messages::support::FieldDescription;

    using utility::math::ransac::Ransac;
    using utility::math::ransac::RansacCircleModel;

    BallDetector::BallDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<BallDetector>>>([this](const Configuration<BallDetector>& config) {
            MINIMUM_POINTS_FOR_CONSENSUS = config["ransac"]["minimum_points_for_consensus"].as<uint>();
            CONSENSUS_ERROR_THRESHOLD = config["ransac"]["consensus_error_threshold"].as<double>();
            MAXIMUM_ITERATIONS_PER_FITTING = config["ransac"]["maximum_iterations_per_fitting"].as<uint>();
            MAXIMUM_FITTED_MODELS = config["ransac"]["maximum_fitted_models"].as<uint>();
            MAXIMUM_DISAGREEMENT_RATIO = config["maximum_disagreement_ratio"].as<double>();
            measurement_distance_variance_factor = config["measurement_distance_variance_factor"].as<double>();
            measurement_bearing_variance = config["measurement_bearing_variance"].as<double>();
            measurement_elevation_variance = config["measurement_elevation_variance"].as<double>();
        });

        on<Trigger<ClassifiedImage<ObjectClass>>, With<CameraParameters>, With<Optional<FieldDescription>>, Options<Single>>("Ball Detector", [this](const ClassifiedImage<ObjectClass>& image, const CameraParameters& cam, const std::shared_ptr<const FieldDescription>& field) {
            if (field == nullptr) {
                NUClear::log(__FILE__, ", ", __LINE__, ": FieldDescription Update: support::configuration::SoccerConfig module might not be installed.");
                throw std::runtime_error("FieldDescription Update: support::configuration::SoccerConfig module might not be installed");
            }
            // This holds our points that may be a part of the ball
            std::vector<arma::vec2> ballPoints;
            const auto& sensors = *image.sensors;

            // Get all the points that could make up the ball
            for(int i = 0; i < 1; ++i) {

                auto segments = i ? image.horizontalSegments.equal_range(ObjectClass::BALL)
                                  : image.verticalSegments.equal_range(ObjectClass::BALL);

                for(auto it = segments.first; it != segments.second; ++it) {

                    auto& segment = it->second;
                    auto& start = segment.start;
                    auto& end = segment.end;

                    bool belowHorizon = image.visualHorizonAtPoint(end[0]) < end[1] || image.visualHorizonAtPoint(start[0]) < start[1];

                    // We throw out points if they are:
                    // Less the full quality (subsampled)
                    // Do not have a transition on either side (are on an edge)
                    // Go from an orange to other to orange segment (are interior)

                    if(belowHorizon
                        && segment.subsample == 1
                        && segment.next
                        && (!segment.next->next || segment.next->next->colour != ObjectClass::BALL)) {

                        ballPoints.push_back({ double(end[0]), double(end[1]) });
                    }

                    if(belowHorizon
                        && segment.subsample == 1
                        && segment.previous
                        && (!segment.previous->previous || segment.previous->previous->colour != ObjectClass::BALL)) {

                        ballPoints.push_back({ double(start[0]), double(start[1]) });
                    }
                }
            }

            // Use ransac to find the ball
            auto ransacResults = Ransac<RansacCircleModel>::fitModels(ballPoints.begin()
                                                                    , ballPoints.end()
                                                                    , MINIMUM_POINTS_FOR_CONSENSUS
                                                                    , MAXIMUM_ITERATIONS_PER_FITTING
                                                                    , MAXIMUM_FITTED_MODELS
                                                                    , CONSENSUS_ERROR_THRESHOLD);

            auto balls = std::make_unique<std::vector<Ball>>();
            balls->reserve(ransacResults.size());

            for(auto& result : ransacResults) {

                std::vector<VisionObject::Measurement> measurements;
                measurements.reserve(2);

                // Transform our centre into kinematics coordinates
                auto centre = imageToScreen(result.model.centre, image.dimensions);

                // Get the 4 points around our circle
                auto top   = centre + arma::vec2({ 0,  result.model.radius });
                auto base  = centre + arma::vec2({ 0, -result.model.radius });
                auto left  = centre + arma::vec2({  result.model.radius, 0 });
                auto right = centre + arma::vec2({ -result.model.radius, 0 });

                double cameraHeight = sensors.orientationCamToGround(2, 3);

                // Get a unit vector pointing to the centre of the ball
                arma::vec3 ballCentreRay = arma::normalise(arma::normalise(getCamFromScreen(top, cam.focalLengthPixels))
                                                           + arma::normalise(getCamFromScreen(base, cam.focalLengthPixels)));

                // Get the centre of our ball ins creen space
                arma::vec2 ballCentreScreen = projectCamSpaceToScreen(ballCentreRay, cam.focalLengthPixels);

                // Get our width based distance to the ball
                double widthDistance = widthBasedDistanceToCircle(field->ball_radius * 2, top, base, cam.focalLengthPixels);
                arma::vec3 cameraPosition = sensors.orientationCamToGround.submat(0,3,2,3);
                arma::vec3 ballCentreGroundWidth = widthDistance * sensors.orientationCamToGround.submat(0,0,2,2) * ballCentreRay + cameraPosition;
                double ballCentreGroundWidthDistance = arma::norm(ballCentreGroundWidth);
                arma::mat ballCentreGroundWidthCov = arma::diagmat(arma::vec({
                    measurement_distance_variance_factor * ballCentreGroundWidthDistance,
                    measurement_bearing_variance,
                    measurement_elevation_variance }));
                measurements.push_back({ cartesianToSpherical(ballCentreGroundWidth), ballCentreGroundWidthCov});
                // 0.003505351, 0.001961638, 1.68276E-05
                emit(graph("ballCentreGroundWidth measurement", ballCentreGroundWidth(0), ballCentreGroundWidth(1), ballCentreGroundWidth(2)));
                emit(graph("ballCentreGroundWidth measurement (spherical)", measurements.back().position(0), measurements.back().position(1), measurements.back().position(2)));

                // Project this vector to a plane midway through the ball
                Plane ballBisectorPlane({ 0, 0, 1 }, { 0, 0, field->ball_radius });
                arma::vec3 ballCentreGroundProj = projectCamToPlane(ballCentreRay, sensors.orientationCamToGround, ballBisectorPlane);
                double ballCentreGroundProjDistance = arma::norm(ballCentreGroundProj);
                arma::mat ballCentreGroundProjCov = arma::diagmat(arma::vec({
                    measurement_distance_variance_factor * ballCentreGroundProjDistance,
                    measurement_bearing_variance,
                    measurement_elevation_variance }));
                measurements.push_back({ cartesianToSpherical(ballCentreGroundProj), ballCentreGroundProjCov});
                // 0.002357231 * 2, 2.20107E-05 * 2, 4.33072E-05 * 2,
                emit(graph("ballCentreGroundProj measurement", ballCentreGroundProj(0), ballCentreGroundProj(1), ballCentreGroundProj(2)));
                emit(graph("ballCentreGroundProj measurement (spherical)", measurements.back().position(0), measurements.back().position(1), measurements.back().position(2)));

                /*
                 *  IF VALID BUILD OUR BALL
                 */
                if(widthDistance > cameraHeight / 2.0 && std::abs((ballCentreGroundWidth[0] - ballCentreGroundProj[0]) / ballCentreGroundProj[0]) > MAXIMUM_DISAGREEMENT_RATIO) {
                    Ball b;

                    // On screen visual shape
                    b.circle.radius = result.model.radius;
                    b.circle.centre = result.model.centre;

                    // Angular positions from the camera
                    b.screenAngular = arma::atan(cam.pixelsToTanThetaFactor % ballCentreScreen);
                    b.angularSize = { getParallaxAngle(left, right, cam.focalLengthPixels), getParallaxAngle(top, base, cam.focalLengthPixels) };

                    // Move our measurements
                    b.measurements = std::move(measurements);

                    b.sensors = image.sensors;
                    balls->push_back(std::move(b));
                }
            }

            for(auto a = balls->begin(); a != balls->end(); ++a) {
                for(auto b = a + 1; b != balls->end();) {

                    // If our balls overlap
                    if(a->circle.distanceToPoint(b->circle.centre) < b->circle.radius) {
                        // Pick the better ball
                        if(a->circle.radius < b->circle.radius) {
                            // Throwout b
                            b = balls->erase(b);
                        }
                        else {
                            a = balls->erase(a);

                            if(a == b) {
                                ++b;
                            }
                        }
                    }
                    else {
                        ++b;
                    }
                }
            }

            emit(std::move(balls));

        });
    }

}
}
