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

#include <Eigen/Core>

#include "extension/Configuration.h"

#include "message/input/CameraParameters.h"
#include "message/support/FieldDescription.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/VisionObjects.h"
#include "message/vision/LookUpTable.h"

#include "utility/math/coordinates.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/geometry/Line.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/ransac/Ransac.h"
#include "utility/math/ransac/RansacCircleModel.h"
#include "utility/math/vision.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/fourcc.h"
#include "utility/vision/Vision.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::input::CameraParameters;

    using message::vision::ClassifiedImage;
    using message::vision::Ball;
    using message::vision::LookUpTable;
    using message::input::Image;
    using message::support::FieldDescription;

    using Plane = utility::math::geometry::Plane<3>;

    using ServoID = utility::input::ServoID;
    using utility::math::vision::widthBasedDistanceToCircle;
    using utility::math::vision::projectCamToPlane;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::getCamFromScreen;
    using utility::math::vision::getParallaxAngle;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::matrix::Transform3D;
    using utility::math::geometry::Circle;
    using utility::math::geometry::Line;

    using utility::math::coordinates::cartesianToSpherical;
    using utility::nubugger::graph;

    using utility::math::ransac::Ransac;
    using utility::math::ransac::RansacCircleModel;
    using utility::nubugger::drawVisionLines;
    using utility::support::Expression;

    using FOURCC = utility::vision::FOURCC;
    using Colour = utility::vision::Colour;

    float BallDetector::approximateCircleGreenRatio(const Circle& circle, const Image& image, const LookUpTable& lut) {
        // TODO:
        // std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>> debug;
        float r = 0;
        int numGreen = 0;
        int actualSamples = 0;
        for(int i = 0; i < green_radial_samples; r = (++i) * circle.radius / float(green_radial_samples)) {
            float theta = 0;
            if(r == 0){
                Eigen::Vector2i ipos = Eigen::VectorXi({int(std::round(circle.centre[0])), int(std::round(circle.centre[1]))});
                if(ipos[0] >= 0 && ipos[0] < int(image.dimensions[0]) && ipos[1] >= 0 && ipos[1] < int(image.dimensions[1])){
                    // debug.push_back(std::make_tuple(ipos, ipos + Eigen::Vector2i{1,1}, Eigen::Vector4d{1,1,1,1}));
                    char c = static_cast<char>(utility::vision::getPixelColour(lut,
                            getPixel(ipos[0], ipos[1], image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format))));
                    if (c == Colour::GREEN) {
                        numGreen++;
                    }
                    actualSamples++;
                }
                continue;
            }
            for(int j = 0; j < green_angular_samples; theta = (++j) * 2 * M_PI / float(green_angular_samples)) {
                float x = r * std::cos(theta);
                float y = r * std::sin(theta);
                Eigen::Vector2d pos = circle.centre + Eigen::Vector2d(x,y);
                Eigen::Vector2i ipos = Eigen::Vector2i({int(std::round(pos[0])),int(std::round(pos[1]))});
                if(ipos[0] >= 0 && ipos[0] < int(image.dimensions[0]) && ipos[1] >= 0 && ipos[1] < int(image.dimensions[1])){
                    // debug.push_back(std::make_tuple(ipos, ipos + Eigen::Vector2i{1,1}, Eigen::Vector4d{1,1,1,1}));
                    char c = static_cast<char>(utility::vision::getPixelColour(lut,
                            getPixel(ipos[0], ipos[1], image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format))));
                    if (c == Colour::GREEN) {
                        numGreen++;
                    }
                    actualSamples++;
                }
            }
            // sample point in lut and check if == Colour::GREEN
        }

        // emit(drawVisionLines(debug));

        float greenRatio = actualSamples == 0 ? 1 : (numGreen / float(actualSamples));
        return greenRatio;
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
        , lastFrame(),
        print_throwout_logs(false) {


        on<Configuration>("BallDetector.yaml").then([this] (const Configuration& config) {

            MINIMUM_POINTS_FOR_CONSENSUS = config["ransac"]["minimum_points_for_consensus"].as<uint>();
            CONSENSUS_ERROR_THRESHOLD = config["ransac"]["consensus_error_threshold"].as<Expression>();

            MAXIMUM_ITERATIONS_PER_FITTING = config["ransac"]["maximum_iterations_per_fitting"].as<uint>();
            MAXIMUM_FITTED_MODELS = config["ransac"]["maximum_fitted_models"].as<uint>();
            MAXIMUM_DISAGREEMENT_RATIO = config["maximum_disagreement_ratio"].as<Expression>();

            maximum_relative_seed_point_distance = config["maximum_relative_seed_point_distance"].as<double>();

            measurement_distance_variance_factor = config["measurement_distance_variance_factor"].as<Expression>();
            measurement_bearing_variance = config["measurement_bearing_variance"].as<Expression>();
            measurement_elevation_variance = config["measurement_elevation_variance"].as<Expression>();

            green_ratio_threshold = config["green_ratio_threshold"].as<Expression>();
            green_radial_samples = config["green_radial_samples"].as<Expression>();
            green_angular_samples = config["green_angular_samples"].as<Expression>();

            kmeansClusterer.configure(config["clustering"]);

            print_throwout_logs = config["print_throwout_logs"].as<bool>();

            lastFrame.time = NUClear::clock::now();
        });

        on<Trigger<ClassifiedImage>
         , With<CameraParameters>
         , With<FieldDescription>
         , With<LookUpTable>
         , Single
         , Priority::LOW>().then("Ball Detector", [this](
            std::shared_ptr<const ClassifiedImage> rawImage
            , const CameraParameters& cam
            , const FieldDescription& field
            , const LookUpTable& lut) {

            const auto& image = *rawImage;
            const auto& sensors = *image.sensors;
            Line horizon(image.horizon.normal, image.horizon.distance);

            // This holds our points that may be a part of the ball
            std::vector<Eigen::Vector2d> ballPoints;
            ballPoints.reserve(image.ballPoints.size());

            for (const auto& point : image.ballPoints) {
                ballPoints.push_back(Eigen::Vector2d(double(point[0]), double(point[1])));
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

            if(print_throwout_logs) log("Ransac : ", ransacResults.size(), "results");

            Eigen::Matrix4d camToGround = convert<double, 4, 4>(sensors.camToGround);

            for (auto& result : ransacResults) {

                // Transform our centre into kinematics coordinates
                Eigen::Vector2d centre = imageToScreen(result.model.centre, convert<uint, 2>(image.dimensions));

                // Get the 4 points around our circle
                Eigen::Vector2d top   = centre + Eigen::Vector2d( 0,  result.model.radius );
                Eigen::Vector2d base  = centre + Eigen::Vector2d( 0, -result.model.radius );
                Eigen::Vector2d left  = centre + Eigen::Vector2d(  result.model.radius, 0 );
                Eigen::Vector2d right = centre + Eigen::Vector2d( -result.model.radius, 0 );

                double cameraHeight = camToGround(2, 3);

                // Get a unit vector pointing to the centre of the ball
                Eigen::Vector3d ballCentreRay = arma::normalise(arma::normalise(getCamFromScreen(top, cam.focalLengthPixels))
                                                           + arma::normalise(getCamFromScreen(base, cam.focalLengthPixels)));

                // Get the centre of our ball in screen space
                Eigen::Vector2d ballCentreScreen = projectCamSpaceToScreen(ballCentreRay, cam.focalLengthPixels);

                /************************************************
                 *                  THROWOUTS                   *
                 ************************************************/
                // CENTRE OF BALL IS ABOVE THE HORIZON
                if(horizon.y(result.model.centre[0]) > result.model.centre[1]) {
                    if(print_throwout_logs) log("Ball discarded: image.horizon.y(result.model.centre[0]) > result.model.centre[1]");
                    continue;
                }

                // DOES HAVE INTERNAL GREEN
                float greenRatio = approximateCircleGreenRatio(result.model, *(image.image), lut);
                if (greenRatio > green_ratio_threshold) {
                    if(print_throwout_logs) log("Ball discarded: greenRatio > green_ratio_threshold");
                    continue;
                }

                // DOES NOT TOUCH 3 SEED POINTS
                Eigen::Vector3d sDist({ std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max() });

                // Loop through our seed points and find the minimum distance one
                for(uint i = 0; i < 3; ++i) {
                    for(auto& s : image.ballSeedPoints[i].points) {
                        double dist = std::fabs(result.model.radius - arma::norm(result.model.centre - Eigen::Vector2d(double(s[0]), double(s[1]))));
                        if(sDist[i] > dist) {
                            sDist[i] = dist;
                        }
                    }
                }
                // Check if our largest one is too far away
                if(arma::max(sDist) / result.model.radius > maximum_relative_seed_point_distance) {
                    if(print_throwout_logs) log("Ball discarded: arma::max(sDist) / result.model.radius > maximum_relative_seed_point_distance");
                    continue;
                }

                // BALL IS CLOSER THAN 1/2 THE HEIGHT OF THE ROBOT BY WIDTH
                double widthDistance = widthBasedDistanceToCircle(field.ball_radius, top, base, cam.focalLengthPixels);
                if(widthDistance < cameraHeight * 0.5) {
                    if(print_throwout_logs) log("Ball discarded: widthDistance < cameraHeight * 0.5");
                    continue;
                }

                // IF THE DISAGREEMENT BETWEEN THE WIDTH AND PROJECTION BASED DISTANCES ARE TOO LARGE
                // Project this vector to a plane midway through the ball
                Plane ballBisectorPlane({ 0, 0, 1 }, { 0, 0, field.ball_radius });
                Eigen::Vector3d ballCentreGroundProj = projectCamToPlane(ballCentreRay, camToGround, ballBisectorPlane);
                double ballCentreGroundProjDistance = arma::norm(ballCentreGroundProj);

                if(std::abs((widthDistance - ballCentreGroundProjDistance) / std::max(ballCentreGroundProjDistance, widthDistance)) > MAXIMUM_DISAGREEMENT_RATIO) {
                    if(print_throwout_logs) log("Ball discarded: Width and proj distance disagree too much: width =", widthDistance, "proj =", ballCentreGroundProjDistance);
                    continue;
                }

                /************************************************
                 *                 MEASUREMENTS                 *
                 ************************************************/

                Ball b;
                b.visObject.sensors = image.sensors;

                // Get our transform to world coordinates
                const Transform3D& Htw = convert<double, 4, 4>(sensors.world);
                const Transform3D& Htc = convert<double, 4, 4>(sensors.forwardKinematics.at(ServoID::HEAD_PITCH));
                Transform3D Hcw = Htc.i() * Htw;
                Transform3D Hwc = Hcw.i();

                // Work out how far away the ball must be to be at the distance it is from the camera
                Eigen::Vector3d width_rBWw = Hwc.transformPoint(ballCentreRay * widthDistance);

                // Put our ball centre projection into the same space
                Eigen::Vector3d proj_rBWw = Hwc.transformPoint(ballCentreGroundProj);

                // Average our two centroids
                Eigen::Vector3d rBWw = (width_rBWw);

                // Attach the position to the object
                b.position = rBWw;

                Transform3D Hgc       = camToGround;
                Eigen::Vector3d width_rBGg = Hgc.transformPoint(ballCentreRay * widthDistance);
                Eigen::Vector3d proj_rBGg  = Hgc.transformPoint(ballCentreGroundProj);
                b.torsoSpacePosition  = width_rBGg;
                // log("ball pos1 =", b.position);
                // log("ball pos2 =", b.torsoSpacePosition);
                // log("width_rBGg =", width_rBGg.t());
                // log("proj_rBGg =", proj_rBGg.t());
                // log("ballCentreRay =",ballCentreRay.t());
                // log("camToGround =\n",camToGround);

                // On screen visual shape
                b.circle.radius = result.model.radius;
                b.circle.centre = result.model.centre;

                // Angular positions from the camera
                b.visObject.screenAngular = arma::atan(convert<double, 2>(cam.pixelsToTanThetaFactor % ballCentreScreen));
                b.visObject.angularSize   << getParallaxAngle(left, right, cam.focalLengthPixels), getParallaxAngle(top, base, cam.focalLengthPixels);

                // Add our points
                for (auto& point : result) {
                    b.edgePoints.push_back(getCamFromScreen(imageToScreen(point, convert<uint, 2>(image.dimensions), cam.focalLengthPixels)));
                }

                balls->push_back(std::move(b));
            }

            for(auto a = balls->begin(); a != balls->end(); ++a) {
                Circle acircle(a->circle.radius, a->circle.centre);

                for(auto b = a + 1; b != balls->end();) {

                    // If our balls overlap
                    if(acircle.distanceToPoint(b->circle.centre) < b->circle.radius) {
                        // Pick the better ball
                        if(acircle.radius < b->circle.radius) {
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
            if(print_throwout_logs) log("Final result: ", balls->size(), "balls");
            emit(std::move(balls));
            lastFrame.time = sensors.timestamp;
        });
    }

}
}
