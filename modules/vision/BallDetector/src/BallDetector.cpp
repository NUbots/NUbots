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

namespace modules {
namespace vision {

    using messages::input::CameraParameters;
    using messages::input::Sensors;

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;
    using messages::vision::Ball;

    using utility::math::vision::widthBasedDistanceToCircle;
    using utility::math::vision::projectCamToGroundPlane;
    using utility::math::vision::getGroundPointFromScreen;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::getCamFromScreen;
    using utility::math::vision::getParallaxAngle;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::vision::projectCamToGroundPlane;
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





                auto centre = result.model.centre;
                auto p1 = centre;
                auto p2 = centre;
                p1[1] += result.model.radius;
                p2[1] -= result.model.radius;

                // Transform p1 p2 to kinematics coordinates
                p1 = imageToScreen(p1, { double(image.dimensions[0]), double(image.dimensions[1]) });
                p2 = imageToScreen(p2, { double(image.dimensions[0]), double(image.dimensions[1]) });

                arma::vec3 camUnitP1 = arma::normalise(getCamFromScreen(p1, cam.focalLengthPixels));
                arma::vec3 camUnitP2 = arma::normalise(getCamFromScreen(p2, cam.focalLengthPixels));
                arma::vec3 ballCentreRay = arma::normalise(0.5 * (camUnitP1 + camUnitP2));
                arma::vec2 ballCentreScreen = projectCamSpaceToScreen(ballCentreRay, cam.focalLengthPixels);

                //Width based method
                double wbd = widthBasedDistanceToCircle(BALL_DIAMETER, p1, p2, cam.focalLengthPixels);
                arma::vec3 ballCentreGroundWidth = wbd * sensors.orientationCamToGround.submat(0,0,2,2) * ballCentreRay + sensors.orientationCamToGround.submat(0,3,2,3);

                //Projection to plane method
                arma::mat44 ballBisectorPlaneTransform = sensors.orientationCamToGround;
                ballBisectorPlaneTransform(2,3) -= BALL_DIAMETER / 2.0;
                arma::vec3 ballCentreGroundProj = arma::vec3({ 0, 0, BALL_DIAMETER / 2.0 }) + projectCamToGroundPlane(ballCentreRay, ballBisectorPlaneTransform);

                //Get angular width
                auto p3 = centre;
                auto p4 = centre;
                p3[0] += result.model.radius;
                p4[0] -= result.model.radius;
                p3 = imageToScreen(p3, { double(image.dimensions[0]), double(image.dimensions[1]) });
                p4 = imageToScreen(p4, { double(image.dimensions[0]), double(image.dimensions[1]) });

                /*
                 *  BUILD OUR BALL
                 */
                Ball b;

                // On screen visual shape
                b.circle.radius = result.model.radius;
                b.circle.centre = result.model.centre;

                // Camera dimensions
                b.screenAngular = arma::atan(cam.pixelsToTanThetaFactor % ballCentreScreen);
                b.angularSize = { getParallaxAngle(p3, p4, cam.focalLengthPixels), getParallaxAngle(p1, p2, cam.focalLengthPixels) };

                b.measurements.push_back({ ballCentreGroundWidth, arma::eye(3,3) });
                b.measurements.push_back({ ballCentreGroundProj, arma::eye(3,3) });

                balls->push_back(std::move(b));

                emit(graph("Width Ball Dist", wbd));
                emit(graph("Width Ball Pos", ballCentreGroundWidth[0],ballCentreGroundWidth[1],ballCentreGroundWidth[2]));
                emit(graph("D2P Ball", ballCentreGroundProj[0], ballCentreGroundProj[1]));
            }

            emit(std::move(balls));

        });
    }

}
}
