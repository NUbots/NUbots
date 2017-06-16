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

#include "GoalDetector.h"

#include "extension/Configuration.h"

#include "RansacGoalModel.h"

#include "message/input/CameraParameters.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/VisionObjects.h"
#include "message/vision/LookUpTable.h"

#include "utility/math/geometry/Quad.h"
#include "utility/math/geometry/Line.h"
#include "utility/math/geometry/Plane.h"

#include "utility/math/ransac/NPartiteRansac.h"
#include "utility/math/vision.h"
#include "utility/math/coordinates.h"

#include "utility/vision/ClassifiedImage.h"
#include "utility/vision/fourcc.h"
#include "utility/vision/LookUpTable.h"
#include "utility/vision/Vision.h"


namespace module {
namespace vision {

    using extension::Configuration;

    using message::input::CameraParameters;

    using utility::math::coordinates::cartesianToSpherical;

    using utility::math::geometry::Line;
    using Plane = utility::math::geometry::Plane<3>;
    using utility::math::geometry::Quad;

    using utility::math::ransac::NPartiteRansac;

    using utility::math::vision::widthBasedDistanceToCircle;
    using utility::math::vision::projectCamToPlane;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::getCamFromScreen;
    using utility::math::vision::getParallaxAngle;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::vision::distanceToVerticalObject;

    using message::vision::LookUpTable;
    using message::vision::ClassifiedImage;
    using SegmentClass = message::vision::ClassifiedImage::SegmentClass::Value;
    using message::vision::Goal;

    // TODO the system is too generous with adding segments above and below the goals and makes them too tall, stop it
    // TODO the system needs to throw out the kinematics and height based measurements when it cannot be sure it saw the tops and bottoms of the goals

    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , MINIMUM_POINTS_FOR_CONSENSUS(0)
        , MAXIMUM_ITERATIONS_PER_FITTING(0)
        , MAXIMUM_FITTED_MODELS(0)
        , CONSENSUS_ERROR_THRESHOLD(0.0)
        , MAXIMUM_ASPECT_RATIO(0.0)
        , MINIMUM_ASPECT_RATIO(0.0)
        , VISUAL_HORIZON_BUFFER(0.0)
        , MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE(0.0)
        , MAXIMUM_ANGLE_BETWEEN_GOALS(0.0)
        , MAXIMUM_VERTICAL_GOAL_PERSPECTIVE_ANGLE(0.0)
        , MEASUREMENT_LIMITS_LEFT(10)
        , MEASUREMENT_LIMITS_RIGHT(10)
        , MEASUREMENT_LIMITS_TOP(10)
        , MEASUREMENT_LIMITS_BASE(10) {


        // Trigger the same function when either update
        on<Configuration, Trigger<CameraParameters>>("GoalDetector.yaml")
        .then([this] (const Configuration& config, const CameraParameters& cam) {

            MINIMUM_POINTS_FOR_CONSENSUS = config["ransac"]["minimum_points_for_consensus"].as<uint>();
            CONSENSUS_ERROR_THRESHOLD = config["ransac"]["consensus_error_threshold"].as<double>();
            MAXIMUM_ITERATIONS_PER_FITTING = config["ransac"]["maximum_iterations_per_fitting"].as<uint>();
            MAXIMUM_FITTED_MODELS = config["ransac"]["maximum_fitted_models"].as<uint>();

            MINIMUM_ASPECT_RATIO = config["aspect_ratio_range"][0].as<double>();
            MAXIMUM_ASPECT_RATIO = config["aspect_ratio_range"][1].as<double>();
            VISUAL_HORIZON_BUFFER = std::max(1, int(cam.focalLengthPixels * tan(config["visual_horizon_buffer"].as<double>())));
            MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE = std::cos(config["minimum_goal_horizon_angle"].as<double>() - M_PI_2);
            MAXIMUM_ANGLE_BETWEEN_GOALS = std::cos(config["maximum_angle_between_goals"].as<double>());
            MAXIMUM_VERTICAL_GOAL_PERSPECTIVE_ANGLE = std::sin(-config["maximum_vertical_goal_perspective_angle"].as<double>());

            MEASUREMENT_LIMITS_LEFT  = config["measurement_limits"]["left"].as<uint>();
            MEASUREMENT_LIMITS_RIGHT = config["measurement_limits"]["right"].as<uint>();
            MEASUREMENT_LIMITS_TOP   = config["measurement_limits"]["top"].as<uint>();
            MEASUREMENT_LIMITS_BASE  = config["measurement_limits"]["base"].as<uint>();
        });

        on<Trigger<ClassifiedImage>
         , With<CameraParameters>
         , With<LookUpTable>
         , Single>().then("Goal Detector", [this] (std::shared_ptr<const ClassifiedImage> rawImage
                          , const CameraParameters& cam
                          , const LookUpTable& lut) {

            const auto& image = *rawImage;
            // Our segments that may be a part of a goal
            std::vector<RansacGoalModel::GoalSegment> segments;
            auto goals = std::make_unique<std::vector<Goal>>();

            // Get our goal segments
            for (const auto& segment : image.horizontalSegments) {

                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on the other side
                if ((segment.segmentClass == SegmentClass::GOAL) && (segment.subsample == 1) && (segment.previous > -1) && (segment.next > -1)) {
                    segments.push_back({ { double(segment.start[0]), double(segment.start[1]) }, { double(segment.end[0]), double(segment.end[1]) } });
                }
            }

            // Partition our segments so that they are split between above and below the horizon
            auto split = std::partition(std::begin(segments), std::end(segments), [image] (const RansacGoalModel::GoalSegment& segment) {
                // Is the midpoint above or below the horizon?
                utility::math::geometry::Line horizon(convert<double, 2>(image.horizon.normal), image.horizon.distance);
                return horizon.distanceToPoint(segment.left + segment.right / 2) > 0;
            });

            // Make an array of our partitions
            std::array<std::vector<RansacGoalModel::GoalSegment>::iterator, RansacGoalModel::REQUIRED_POINTS + 1> points = {
                segments.begin(),
                split,
                segments.end()
            };

            // Ransac for goals
            auto models = NPartiteRansac<RansacGoalModel>::fitModels(points
                                                                   , MINIMUM_POINTS_FOR_CONSENSUS
                                                                   , MAXIMUM_ITERATIONS_PER_FITTING
                                                                   , MAXIMUM_FITTED_MODELS
                                                                   , CONSENSUS_ERROR_THRESHOLD);

            // Look at our results
            for (auto& result : models) {

                // Get our left, right and midlines
                Line& left = result.model.left;
                Line& right = result.model.right;
                Line mid;

                // Normals in same direction
                if(arma::dot(left.normal, right.normal) > 0) {
                    mid.normal = arma::normalise(right.normal + left.normal);
                    mid.distance = ((right.distance / arma::dot(right.normal, mid.normal)) + (left.distance / arma::dot(left.normal, mid.normal))) * 0.5;
                }
                // Normals opposed
                else {
                    mid.normal = arma::normalise(right.normal - left.normal);
                    mid.distance = ((right.distance / arma::dot(right.normal, mid.normal)) - (left.distance / arma::dot(left.normal, mid.normal))) * 0.5;
                }

                // Find a point that should work to start searching down
                arma::vec2 midpoint({0, 0});
                int i = 0;
                for(auto& m : result) {
                    midpoint += m.left;
                    midpoint += m.right;
                    i += 2;
                }
                midpoint /= i;

                // Work out which direction to go
                arma::vec2 direction = mid.tangent();
                direction *= direction[1] > 0 ? 1 : -1;
                double theta = std::acos(direction[0]);
                if (std::abs(theta) < M_PI_4) {
                    direction[0] = 1;
                    direction[1] = -std::tan(theta);
                }
                else {
                    direction[0] = std::tan(M_PI_2 - std::abs(theta));
                    direction[1] = 1;
                }

                // Classify until we reach green
                arma::vec2 basePoint({0, 0});
                int notWhiteLen = 0;
                for(arma::vec2 point = mid.orthogonalProjection(midpoint);
                    (point[0] < image.dimensions[0]) && (point[0] > 0) && (point[1] < image.dimensions[1]);
                    point += direction) {

                    char c = static_cast<char>(utility::vision::getPixelColour(lut, 
                        utility::vision::getPixel(int(point[0]), int(point[1]), image.image->dimensions[0], image.image->dimensions[1], image.image->data, 
                                                    static_cast<utility::vision::FOURCC>(image.image->format))));

                    if(c != 'y') {
                        ++notWhiteLen;
                        if(notWhiteLen > 4) {
                            basePoint = point;
                            break;
                        }
                    }
                    else if(c == 'g') {
                        basePoint = point;
                        break;
                    }
                    else if(c == 'y') {
                        notWhiteLen = 0;
                    }
                }

                arma::running_stat<double> stat;

                // Look through our segments to find endpoints
                for(auto& point : result) {
                    // Project left and right onto midpoint keep top and bottom
                    stat(mid.tangentialDistanceToPoint(point.left));
                    stat(mid.tangentialDistanceToPoint(point.right));
                }

                // Get our endpoints from the min and max points on the line
                arma::vec2 midP1 = mid.pointFromTangentialDistance(stat.min());
                arma::vec2 midP2 = mid.orthogonalProjection(basePoint);

                // Project those points outward onto the quad
                arma::vec2 p1 = midP1 - left.distanceToPoint(midP1)  * arma::dot(left.normal, mid.normal)  * mid.normal;
                arma::vec2 p2 = midP2 - left.distanceToPoint(midP2)  * arma::dot(left.normal, mid.normal)  * mid.normal;
                arma::vec2 p3 = midP1 - right.distanceToPoint(midP1) * arma::dot(right.normal, mid.normal) * mid.normal;
                arma::vec2 p4 = midP2 - right.distanceToPoint(midP2) * arma::dot(right.normal, mid.normal) * mid.normal;

                // Make a quad
                Goal goal;
                goal.visObject.sensors = image.sensors;
                goal.side = Goal::Side::UNKNOWN_SIDE;

                // Seperate tl and bl
                arma::vec2 tl = p1[1] > p2[1] ? p2 : p1;
                arma::vec2 bl = p1[1] > p2[1] ? p1 : p2;
                arma::vec2 tr = p3[1] > p4[1] ? p4 : p3;
                arma::vec2 br = p3[1] > p4[1] ? p3 : p4;

                goal.quad.bl = convert<double, 2>(bl);
                goal.quad.tl = convert<double, 2>(tl);
                goal.quad.tr = convert<double, 2>(tr);
                goal.quad.br = convert<double, 2>(br);

                goals->push_back(std::move(goal));
            }

            utility::math::geometry::Line horizon(convert<double, 2>(image.horizon.normal), image.horizon.distance);

            // Throwout invalid quads
            for(auto it = goals->begin(); it != goals->end();) {

                utility::math::geometry::Quad quad(convert<double, 2>(it->quad.bl),
                                                   convert<double, 2>(it->quad.tl),
                                                   convert<double, 2>(it->quad.tr),
                                                   convert<double, 2>(it->quad.br));

                arma::vec2 lhs = arma::normalise(quad.getTopLeft()  - quad.getBottomLeft());
                arma::vec2 rhs = arma::normalise(quad.getTopRight() - quad.getBottomRight());

                // Check if we are within the aspect ratio range
                bool valid = quad.aspectRatio() > MINIMUM_ASPECT_RATIO
                          && quad.aspectRatio() < MAXIMUM_ASPECT_RATIO

                // Check if we are close enough to the visual horizon
                          && (utility::vision::visualHorizonAtPoint(image, quad.getBottomLeft()[0]) < quad.getBottomLeft()[1] + VISUAL_HORIZON_BUFFER
                              || utility::vision::visualHorizonAtPoint(image, quad.getBottomRight()[0]) < quad.getBottomRight()[1] + VISUAL_HORIZON_BUFFER)

                // Check we finish above the kinematics horizon or or kinematics horizon is off the screen
                          && (horizon.y(quad.getTopLeft()[0])  > quad.getTopLeft()[1]  || horizon.y(quad.getTopLeft()[0])  < 0)
                          && (horizon.y(quad.getTopRight()[0]) > quad.getTopRight()[1] || horizon.y(quad.getTopRight()[0]) < 0)

                // Check that our two goal lines are perpendicular with the horizon must use greater than rather then less than because of the cos
                          && std::abs(arma::dot(rhs, horizon.normal)) > MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE
                          && std::abs(arma::dot(lhs, horizon.normal)) > MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE

                // Check that our two goal lines are approximatly parallel
                          && std::abs(arma::dot(lhs, rhs)) > MAXIMUM_ANGLE_BETWEEN_GOALS;

                // Check that our goals don't form too much of an upward cup (not really possible for us)
                          //&& lhs.at(0) * rhs.at(1) - lhs.at(1) * rhs.at(0) > MAXIMUM_VERTICAL_GOAL_PERSPECTIVE_ANGLE;


                if(!valid) {
                    it = goals->erase(it);
                }
                else {
                    ++it;
                }
            }

            // Merge close goals
            for (auto a = goals->begin(); a != goals->end(); ++a) {
                utility::math::geometry::Quad aquad(convert<double, 2>(a->quad.bl),
                                                    convert<double, 2>(a->quad.tl),
                                                    convert<double, 2>(a->quad.tr),
                                                    convert<double, 2>(a->quad.br));

                for (auto b = std::next(a); b != goals->end();) {

                    utility::math::geometry::Quad bquad(convert<double, 2>(b->quad.bl),
                                                        convert<double, 2>(b->quad.tl),
                                                        convert<double, 2>(b->quad.tr),
                                                        convert<double, 2>(b->quad.br));


                    if (aquad.overlapsHorizontally(bquad)) {
                        // Get outer lines.
                        arma::vec2 tl, tr, bl, br;

                        tl = { std::min(aquad.getTopLeft()[0],     bquad.getTopLeft()[0]),     std::min(aquad.getTopLeft()[1],     bquad.getTopLeft()[1]) };
                        tr = { std::max(aquad.getTopRight()[0],    bquad.getTopRight()[0]),    std::min(aquad.getTopRight()[1],    bquad.getTopRight()[1]) };
                        bl = { std::min(aquad.getBottomLeft()[0],  bquad.getBottomLeft()[0]),  std::max(aquad.getBottomLeft()[1],  bquad.getBottomLeft()[1]) };
                        br = { std::max(aquad.getBottomRight()[0], bquad.getBottomRight()[0]), std::max(aquad.getBottomRight()[1], bquad.getBottomRight()[1]) };

                        // Replace original two quads with the new one.
                        aquad.set(bl, tl, tr, br);
                        a->quad.bl = convert<double, 2>(bl);
                        a->quad.tl = convert<double, 2>(tl);
                        a->quad.tr = convert<double, 2>(tr);
                        a->quad.br = convert<double, 2>(br);
                        b = goals->erase(b);
                    }
                    else {
                        b++;
                    }
                }
            }

            // Store our measurements
            for(auto it = goals->begin(); it != goals->end(); ++it) {
                utility::math::geometry::Quad quad(convert<double, 2>(it->quad.bl),
                                                   convert<double, 2>(it->quad.tl),
                                                   convert<double, 2>(it->quad.tr),
                                                   convert<double, 2>(it->quad.br));

                // Get the quad points in screen coords
                arma::vec2 tl = imageToScreen(quad.getTopLeft(),     convert<uint, 2>(image.dimensions));
                arma::vec2 tr = imageToScreen(quad.getTopRight(),    convert<uint, 2>(image.dimensions));
                arma::vec2 bl = imageToScreen(quad.getBottomLeft(),  convert<uint, 2>(image.dimensions));
                arma::vec2 br = imageToScreen(quad.getBottomRight(), convert<uint, 2>(image.dimensions));
                arma::vec2 screenGoalCentre = (tl + tr + bl + br) * 0.25;

                // Get vectors for TL TR BL BR;
                arma::vec3 ctl = getCamFromScreen(tl, cam.focalLengthPixels);
                arma::vec3 ctr = getCamFromScreen(tr, cam.focalLengthPixels);
                arma::vec3 cbl = getCamFromScreen(bl, cam.focalLengthPixels);
                arma::vec3 cbr = getCamFromScreen(br, cam.focalLengthPixels);

                // Get our four normals for each edge
                // BL TL cross product gives left side
                auto left   = convert<double, 3>(arma::normalise(arma::cross(cbl, ctl)));
                it->measurement.push_back(Goal::Measurement(Goal::MeasurementType::LEFT_NORMAL, left));

                // TR BL cross product gives right side
                auto right  = convert<double, 3>(arma::normalise(arma::cross(ctr, cbl)));
                it->measurement.push_back(Goal::Measurement(Goal::MeasurementType::RIGHT_NORMAL, right));

                // Check that the points are not too close to the edges of the screen
                if(                         std::min(cbr[0], cbl[0]) > MEASUREMENT_LIMITS_LEFT
                &&                          std::min(cbr[1], cbl[1]) > MEASUREMENT_LIMITS_TOP
                && cam.imageSizePixels[0] - std::max(cbr[0], cbl[0]) < MEASUREMENT_LIMITS_TOP
                && cam.imageSizePixels[1] - std::max(cbr[1], cbl[1]) < MEASUREMENT_LIMITS_BASE) {

                    // BR BL cross product gives the bottom side
                    auto bottom = convert<double, 3>(arma::normalise(arma::cross(cbr, cbl)));
                    it->measurement.push_back(Goal::Measurement(Goal::MeasurementType::BASE_NORMAL, bottom));
                }

                // Check that the points are not too close to the edges of the screen
                if(                         std::min(ctr[0], ctl[0]) > MEASUREMENT_LIMITS_LEFT
                &&                          std::min(ctr[1], ctl[1]) > MEASUREMENT_LIMITS_TOP
                && cam.imageSizePixels[0] - std::max(ctr[0], ctl[0]) < MEASUREMENT_LIMITS_TOP
                && cam.imageSizePixels[1] - std::max(ctr[1], ctl[1]) < MEASUREMENT_LIMITS_BASE) {

                    // TL TR cross product gives the top side
                    auto top    = convert<double, 3>(arma::normalise(arma::cross(ctl, ctr)));
                    it->measurement.push_back(Goal::Measurement(Goal::MeasurementType::TOP_NORMAL, top));
                }

                // Angular positions from the camera
                arma::vec2 pixelsToTanThetaFactor = convert<double, 2>(cam.pixelsToTanThetaFactor);
                it->visObject.screenAngular = convert<double, 2>(arma::atan(pixelsToTanThetaFactor % screenGoalCentre));
                arma::vec2 brAngular = arma::atan(pixelsToTanThetaFactor % br);
                arma::vec2 trAngular = arma::atan(pixelsToTanThetaFactor % tr);
                arma::vec2 blAngular = arma::atan(pixelsToTanThetaFactor % bl);
                arma::vec2 tlAngular = arma::atan(pixelsToTanThetaFactor % tl);
                Quad angularQuad(blAngular,tlAngular,trAngular,brAngular);
                it->visObject.angularSize = convert<double, 2>(angularQuad.getSize());
            }


            // Assign leftness and rightness to goals
            if (goals->size() == 2) {
                utility::math::geometry::Quad quad0(convert<double, 2>(goals->at(0).quad.bl),
                                                    convert<double, 2>(goals->at(0).quad.tl),
                                                    convert<double, 2>(goals->at(0).quad.tr),
                                                    convert<double, 2>(goals->at(0).quad.br));

                utility::math::geometry::Quad quad1(convert<double, 2>(goals->at(1).quad.bl),
                                                    convert<double, 2>(goals->at(1).quad.tl),
                                                    convert<double, 2>(goals->at(1).quad.tr),
                                                    convert<double, 2>(goals->at(1).quad.br));

                if (quad0.getCentre()(0) < quad1.getCentre()(0)) {
                    goals->at(0).side = Goal::Side::LEFT;
                    goals->at(1).side = Goal::Side::RIGHT;
                } else {
                    goals->at(0).side = Goal::Side::RIGHT;
                    goals->at(1).side = Goal::Side::LEFT;
                }
            }

            emit(std::move(goals));

        });
    }

}
}

