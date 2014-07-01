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

#include "utility/math/ransac/RansacCircleModel.h"
#include "utility/math/vision.h"
#include "utility/nubugger/NUgraph.h"

namespace modules {
namespace vision {

    using utility::math::ransac::RansacCircleModel;
    using utility::math::ransac::findMultipleModels;
    using utility::math::ransac::RansacSelectionMethod;

    using messages::input::CameraParameters;
    using messages::input::Sensors;

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;
    using messages::vision::Ball;

    using utility::math::vision::distanceToEquidistantPoints;
    using utility::math::vision::getGroundPointMeanOfEquidistantPoints;
    using utility::math::vision::getGroundPointFromScreen;
    using utility::math::vision::imageToCam;
    using utility::nubugger::graph;

    using messages::support::Configuration;

    BallDetector::BallDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<BallDetector>>>([this](const Configuration<BallDetector>& config) {

            std::string selectionMethod = config["SELECTION_METHOD"].as<std::string>();

            if (selectionMethod.compare("LARGEST_CONSENSUS") == 0) {
                SELECTION_METHOD = RansacSelectionMethod::LARGEST_CONSENSUS;
            }
            else if (selectionMethod.compare("BEST_FITTING_CONSENSUS") == 0) {
                SELECTION_METHOD = RansacSelectionMethod::BEST_FITTING_CONSENSUS;
            }
            else {
                SELECTION_METHOD = RansacSelectionMethod::LARGEST_CONSENSUS;
            }

            MINIMUM_POINTS = config["MINIMUM_POINTS"].as<uint>();
            CONSENSUS_THRESHOLD = config["CONSENSUS_THRESHOLD"].as<double>();
            MAX_ITERATIONS_PER_FITTING = config["MAX_ITERATIONS_PER_FITTING"].as<uint>();
            MAX_FITTING_ATTEMPTS = config["MAX_FITTING_ATTEMPTS"].as<uint>();
        });


        on<Trigger<ClassifiedImage<ObjectClass>>, With<CameraParameters, Sensors>>([this](const ClassifiedImage<ObjectClass>& image, const CameraParameters& cam, const Sensors& sensors) {

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
            auto ransacResults = findMultipleModels<RansacCircleModel<arma::vec2>, arma::vec2>(ballPoints,
                                                                                               CONSENSUS_THRESHOLD,
                                                                                               MINIMUM_POINTS,
                                                                                               MAX_ITERATIONS_PER_FITTING,
                                                                                               MAX_FITTING_ATTEMPTS,
                                                                                               SELECTION_METHOD);

            auto balls = std::make_unique<std::vector<Ball>>();
            balls->reserve(ransacResults.size());

            for(auto& ball : ransacResults) {


                auto centre = ball.first.getCentre();
                auto p1 = centre;
                auto p2 = centre;
                p1[1] += ball.first.getRadius();
                p2[1] -= ball.first.getRadius();

                // Transform p1 p2 to kinematics coordinates
                p1 = imageToCam(p1, { double(image.dimensions[0]), double(image.dimensions[1]) });
                p2 = imageToCam(p2, { double(image.dimensions[0]), double(image.dimensions[1]) });

                double wbd = distanceToEquidistantPoints(0.10, p1, p2, cam.focalLengthPixels);
                arma::vec3 wb = getGroundPointMeanOfEquidistantPoints(0.10, p1, p2, cam.focalLengthPixels, sensors.orientationCamToGround);
                auto d2p = getGroundPointFromScreen(p1, sensors.orientationCamToGround, cam.focalLengthPixels);
                std::cout << "Width: " << wbd << std::endl;
                std::cout << "D2P: " << d2p.t() << std::endl;

                emit(graph("Width Ball Dist", wbd));
                emit(graph("Width Ball Pos", wb[0],wb[1],wb[2]));
                emit(graph("D2P Ball", d2p[0], d2p[1]));

                // TODO fuse the width and point based distances

                Ball b;

                b.circle.radius = ball.first.getRadius();
                b.circle.centre = ball.first.getCentre();

                balls->push_back(std::move(b));
            }

            emit(std::move(balls));

        });
    }

}
}
