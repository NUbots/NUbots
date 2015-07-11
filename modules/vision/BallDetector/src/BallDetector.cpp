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
#include "messages/vision/LookUpTable.h"
#include "messages/input/CameraParameters.h"
#include "messages/support/Configuration.h"
#include "messages/support/FieldDescription.h"

#include "utility/support/yaml_expression.h"

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
    using messages::vision::LookUpTable;
    using messages::input::Image;

    using Plane = utility::math::geometry::Plane<3>;

    using utility::math::vision::widthBasedDistanceToCircle;
    using utility::math::vision::projectCamToPlane;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::getCamFromScreen;
    using utility::math::vision::getParallaxAngle;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::geometry::Circle;

    using utility::math::coordinates::cartesianToSpherical;
    using utility::nubugger::graph;

    using messages::support::Configuration;
    using messages::support::FieldDescription;

    using utility::math::ransac::Ransac;
    using utility::math::ransac::RansacCircleModel;
    using utility::nubugger::drawVisionLines;
    using utility::support::Expression;

    float BallDetector::approximateCircleGreenRatio(const Circle& circle, const Image& image, const LookUpTable& lut) {
        // TODO:
        // std::vector<std::tuple<arma::ivec2, arma::ivec2, arma::vec4>> debug;
        float r = 0;
        int numGreen = 0;
        for(int i = 0; i < green_radial_samples; r = (++i) * circle.radius / float(green_radial_samples)) {
            float theta = 0;
            if(r == 0){
                arma::ivec2 ipos = arma::ivec({int(std::round(circle.centre[0])), int(std::round(circle.centre[1]))});
                if(ipos[0] >= 0 && ipos[0] < image.width && ipos[1] >= 0 && ipos[1] < image.height){
                    // debug.push_back(std::make_tuple(ipos, ipos + arma::ivec2{1,1}, arma::vec4{1,1,1,1}));
                    if(lut(image(ipos)) == 'g'){
                        numGreen++;
                    }
                }
                continue;
            }
            for(int j = 0; j < green_angular_samples; theta = (++j) * 2 * M_PI / float(green_angular_samples)) {
                float x = r * std::cos(theta);
                float y = r * std::sin(theta);
                arma::vec2 pos = circle.centre + arma::vec2({x,y});
                arma::ivec2 ipos = arma::ivec2({int(std::round(pos[0])),int(std::round(pos[1]))});
                if(ipos[0] >= 0 && ipos[0] < image.width && ipos[1] >= 0 && ipos[1] < image.height){
                    // debug.push_back(std::make_tuple(ipos, ipos + arma::ivec2{1,1}, arma::vec4{1,1,1,1}));
                    if(lut(image(ipos)) == 'g'){
                        numGreen++;
                    }
                }
            }
            // sample point in lut and check if == 'g'
        }

        // emit(drawVisionLines(debug));

        float greenRatio = numGreen / float(1 + (green_radial_samples-1) * green_angular_samples);
        return greenRatio;
    }

    BallDetector::BallDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<BallDetector>>>([this](const Configuration<BallDetector>& config) {
            
            MINIMUM_POINTS_FOR_CONSENSUS = config["ransac"]["minimum_points_for_consensus"].as<uint>();
            CONSENSUS_ERROR_THRESHOLD = config["ransac"]["consensus_error_threshold"].as<Expression>();
            
            MAXIMUM_ITERATIONS_PER_FITTING = config["ransac"]["maximum_iterations_per_fitting"].as<uint>();
            MAXIMUM_FITTED_MODELS = config["ransac"]["maximum_fitted_models"].as<uint>();
            MAXIMUM_DISAGREEMENT_RATIO = config["maximum_disagreement_ratio"].as<Expression>();
            
            measurement_distance_variance_factor = config["measurement_distance_variance_factor"].as<Expression>();
            measurement_bearing_variance = config["measurement_bearing_variance"].as<Expression>();
            measurement_elevation_variance = config["measurement_elevation_variance"].as<Expression>();
            
            green_ratio_threshold = config["green_ratio_threshold"].as<Expression>();
            green_radial_samples = config["green_radial_samples"].as<Expression>();
            green_angular_samples = config["green_angular_samples"].as<Expression>();

            kmeansClusterer.configure(config["clustering"]);

            lastFrame.time = NUClear::clock::now();
        });

        on<Trigger<Raw<ClassifiedImage<ObjectClass>>>, With<CameraParameters>, With<Optional<FieldDescription>>, With<LookUpTable>, Options<Single>>("Ball Detector", [this](
            const std::shared_ptr<const ClassifiedImage<ObjectClass>>& rawImage, const CameraParameters& cam, const std::shared_ptr<const FieldDescription>& field, const LookUpTable& lut) {

            if (field == nullptr) {
                NUClear::log(__FILE__, ", ", __LINE__, ": FieldDescription Update: support::configuration::SoccerConfig module might not be installed.");
                throw std::runtime_error("FieldDescription Update: support::configuration::SoccerConfig module might not be installed");
            }
            const auto& image = *rawImage;
            // This holds our points that may be a part of the ball
            std::vector<arma::vec2> ballPoints;
            const auto& sensors = *image.sensors;

            ballPoints.reserve(image.ballPoints.size());
            for(const auto& point : image.ballPoints) {
                ballPoints.push_back(arma::vec2({double(point[0]), double(point[1])}));
            }

            double deltaT = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(sensors.timestamp - lastFrame.time).count();
            
            // //Cluster data points for running ransac
            // arma::mat clusterData = arma::zeros(2,ballPoints.size());
            // for (int i = 0; i < ballPoints.size(); i++){
            //     clusterData.col(i) = ballPoints[i];
            // }
            // bool clusterSuccess = kmeansClusterer.learn(clusterData);

            // // std::vector<RansacResult<std::iterator, RansacCircleModel>> ransacResults;

            // if(clusterSuccess){
            //     std::cout << "Cluster success!" << std::endl;
            //     // Do ransac per cluster 
            //     auto debug = kmeansClusterer.getDebugRectangles();
            //     emit(drawVisionLines(debug));
            // } else {
            // }
            
            auto ransacResults = Ransac<RansacCircleModel>::fitModels(ballPoints.begin()
                                                                    , ballPoints.end()
                                                                    , MINIMUM_POINTS_FOR_CONSENSUS
                                                                    , MAXIMUM_ITERATIONS_PER_FITTING
                                                                    , MAXIMUM_FITTED_MODELS
                                                                    , CONSENSUS_ERROR_THRESHOLD);
            // Use ransac to find the ball

            auto balls = std::make_unique<std::vector<Ball>>();
            balls->reserve(ransacResults.size());

            for(auto& result : ransacResults) {
                float greenRatio = approximateCircleGreenRatio(result.model, *(image.image), lut);
                if(greenRatio > green_ratio_threshold){
                    continue;
                }

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

                // Get the centre of our ball in screen space
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

                //compute velocity
                arma::mat widthVelCov = measurement_distance_variance_factor * ballCentreGroundWidthDistance * arma::eye(3,3);
                arma::vec3 widthVel;
                if(deltaT == 0) {
                    widthVel = arma::zeros(3);
                    widthVelCov = 1e5 * arma::eye(3,3);
                }else if(deltaT < 1){
                    widthVel = (ballCentreGroundWidth - lastFrame.widthBall) / deltaT;
                } else {
                    //If we haven't see the ball for a while we don't measure velocity
                    widthVel = arma::zeros(3);
                    log<NUClear::WARN>("Ball velocity frame dropped because of too much time between frames");
                }
                lastFrame.widthBall = ballCentreGroundWidth;
                //push back measurements
                arma::vec3 sphericalBallCentreGroundWidth = cartesianToSpherical(ballCentreGroundWidth);
                measurements.push_back({ sphericalBallCentreGroundWidth, ballCentreGroundWidthCov, widthVel, widthVelCov});
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
                //compute velocity
                arma::mat projVelCov = measurement_distance_variance_factor * ballCentreGroundWidthDistance * arma::eye(3,3);
                arma::vec3 projVel;
                if(deltaT == 0) {
                    projVel = arma::zeros(3);
                    projVelCov = 1e5 * arma::eye(3,3);
                }else if(deltaT < 1){
                    projVel = (ballCentreGroundProj - lastFrame.projBall) / deltaT;
                } else {
                    //If we haven't see the ball for a while we don't measure velocity
                    projVel = arma::zeros(3);
                    log<NUClear::WARN>("Ball velocity frame dropped because of too much time between frames");
                }
                lastFrame.projBall = ballCentreGroundProj;
                //push back measurements
                arma::vec3 sphericalBallCentreGroundProj = cartesianToSpherical(ballCentreGroundProj);
                measurements.push_back({ sphericalBallCentreGroundProj, ballCentreGroundProjCov, projVel, projVelCov});
                // 0.002357231 * 2, 2.20107E-05 * 2, 4.33072E-05 * 2,
                emit(graph("ballCentreGroundProj measurement", ballCentreGroundProj(0), ballCentreGroundProj(1), ballCentreGroundProj(2)));
                emit(graph("ballCentreGroundProj measurement (spherical)", measurements.back().position(0), measurements.back().position(1), measurements.back().position(2)));

                /*
                 *  IF VALID BUILD OUR BALL
                 */
                if(widthDistance > cameraHeight / 2.0 
                    //Only build ball if disagreement not too high
                    && std::abs((sphericalBallCentreGroundWidth[0] - sphericalBallCentreGroundProj[0]) / sphericalBallCentreGroundProj[0]) < MAXIMUM_DISAGREEMENT_RATIO) {
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
                    b.classifiedImage = rawImage;
                    balls->push_back(std::move(b));
                }
            }

            for(auto a = balls->begin(); a != balls->end(); ++a) {
                for(auto b = a + 1; b != balls->end();) {

                    // If our balls overlap
                    if(a->circle.distanceToPoint(b->circle.centre) < b->circle.radius) {
                        // Pick the better ball
                        if(a->circle.radius < b->circle.radius) {
                            // Throw-out b
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

            lastFrame.time = sensors.timestamp;

        });
    }

}
}
