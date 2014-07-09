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

#include "messages/input/Sensors.h"
#include "messages/vision/ClassifiedImage.h"
#include "messages/support/Configuration.h"
#include "messages/vision/VisionObjects.h"
#include "messages/input/CameraParameters.h"

#include "utility/math/ransac/Ransac.h"
#include "utility/math/ransac/RansacCircleModel.h"
#include "utility/math/vision.h"
#include "utility/nubugger/NUgraph.h"
#include "utility/math/coordinates.h"

namespace modules {
namespace vision {

    using messages::input::CameraParameters;
    using messages::input::Sensors;

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;
    using messages::vision::VisionObject;
    using messages::vision::Ball;

    using utility::math::vision::widthBasedDistanceToCircle;
    using utility::math::vision::projectCamToGroundPlane;
    using utility::math::vision::getGroundPointFromScreen;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::getCamFromScreen;
    using utility::math::vision::getParallaxAngle;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::vision::projectCamToGroundPlane;

    using utility::math::coordinates::cartesianToSpherical;
    using utility::nubugger::graph;

    using messages::support::Configuration;

    using utility::math::ransac::Ransac;
    using utility::math::ransac::RansacCircleModel;

    BallDetector::BallDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<BallDetector>>>([this](const Configuration<BallDetector>& config) {
            MINIMUM_POINTS_FOR_CONSENSUS = config["ransac"]["minimum_points_for_consensus"].as<uint>();
            CONSENSUS_ERROR_THRESHOLD = config["ransac"]["consensus_error_threshold"].as<double>();
            MAXIMUM_ITERATIONS_PER_FITTING = config["ransac"]["maximum_iterations_per_fitting"].as<uint>();
            MAXIMUM_FITTED_MODELS = config["ransac"]["maximum_fitted_models"].as<uint>();
        });


        on<Trigger<ClassifiedImage<ObjectClass>>, With<CameraParameters, Sensors>, Options<Single>>("Ball Detector", [this](const ClassifiedImage<ObjectClass>& image, const CameraParameters& cam, const Sensors& sensors) {

            // This holds our points that may be a part of the ball
            std::vector<arma::vec2> ballPoints;

            // Get all the points that could make up the ball
            for(int i = 0; i < 1; ++i) {

                auto segments = i ? image.horizontalSegments.equal_range(ObjectClass::BALL)
                                  : image.verticalSegments.equal_range(ObjectClass::BALL);

                for(auto it = segments.first; it != segments.second; ++it) {

                    auto& start = it->second.start;
                    auto& end = it->second.end;

                    // We throw out points if they are:
                    // Less the full quality (subsampled)
                    // Do not have a transition on either side (are on an edge)

                    if(it->second.subsample == 1 && it->second.next && image.visualHorizonAtPoint(end[0]) < end[1]) {

                        ballPoints.push_back({ double(it->second.end[0]), double(it->second.end[1]) });
                    }

                    if(it->second.subsample == 1 && it->second.previous && image.visualHorizonAtPoint(start[0]) < start[1]) {

                        ballPoints.push_back({ double(it->second.start[0]), double(it->second.start[1]) });
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

                double BALL_DIAMETER = 0.1; //TODO:Universal CONFIG

                std::vector<VisionObject::Measurement> measurements;
                measurements.reserve(2);

                // Transform our centre into kinematics coordinates
                auto centre = imageToScreen(result.model.centre, image.dimensions);

                // Get the 4 points around our circle
                auto top   = centre + arma::vec2({ 0,  result.model.radius });
                auto base  = centre + arma::vec2({ 0, -result.model.radius });
                auto left  = centre + arma::vec2({  result.model.radius, 0 });
                auto right = centre + arma::vec2({ -result.model.radius, 0 });

                // Get a unit vector pointing to the centre of the ball
                arma::vec3 ballCentreRay = arma::normalise(arma::normalise(getCamFromScreen(top, cam.focalLengthPixels))
                                                           + arma::normalise(getCamFromScreen(base, cam.focalLengthPixels)));
                // Get the centre of our ball ins creen space
                arma::vec2 ballCentreScreen = projectCamSpaceToScreen(ballCentreRay, cam.focalLengthPixels);

                // Get our width based distance to the ball
                double widthDistance = widthBasedDistanceToCircle(BALL_DIAMETER, top, base, cam.focalLengthPixels);
                arma::vec3 ballCentreGroundWidth = widthDistance * sensors.orientationCamToGround.submat(0,0,2,2) * ballCentreRay + sensors.orientationCamToGround.submat(0,3,2,3);

                measurements.push_back({ cartesianToSpherical(ballCentreGroundWidth), arma::diagmat(arma::vec({0.003505351, 0.001961638, 1.68276E-05})) });

                // Project this vector to a plane midway through the ball
                arma::mat44 ballBisectorPlaneTransform = sensors.orientationCamToGround;
                ballBisectorPlaneTransform(2,3) -= BALL_DIAMETER / 2.0;
                arma::vec3 ballCentreGroundProj = arma::vec3({ 0, 0, BALL_DIAMETER / 2.0 }) + projectCamToGroundPlane(ballCentreRay, ballBisectorPlaneTransform);

                measurements.push_back({ cartesianToSpherical(ballCentreGroundProj), arma::diagmat(arma::vec({0.002357231 * 2, 2.20107E-05 * 2, 4.33072E-05 * 2 })) });

                // std::cerr << measurements[0].position[0]
                //    << "," << measurements[0].position[1]
                //    << "," << measurements[0].position[2]
                //    << "," << measurements[1].position[0]
                //    << "," << measurements[1].position[1]
                //    << "," << measurements[1].position[2]
                //    << std::endl;

                /*
                 *  BUILD OUR BALL
                 */
                Ball b;

                // On screen visual shape
                b.circle.radius = result.model.radius;
                b.circle.centre = result.model.centre;

                // Angular positions from the camera
                b.screenAngular = arma::atan(cam.pixelsToTanThetaFactor % ballCentreScreen);
                b.angularSize = { getParallaxAngle(left, right, cam.focalLengthPixels), getParallaxAngle(top, base, cam.focalLengthPixels) };

                // Move our measurements
                b.measurements = std::move(measurements);

                balls->push_back(std::move(b));
            }

            emit(std::move(balls));

        });
    }

}
}
