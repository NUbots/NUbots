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

#include "RansacGoalModel.h"

#include "message/vision/ClassifiedImage.h"
#include "message/vision/VisionObjects.h"
#include "message/vision/LookUpTable.h"
#include "message/support/Configuration.h"
#include "message/support/FieldDescription.h"

#include "utility/math/geometry/Quad.h"
#include "utility/math/geometry/Line.h"
#include "utility/math/geometry/Plane.h"

#include "utility/math/ransac/NPartiteRansac.h"
#include "utility/math/vision.h"
#include "utility/math/coordinates.h"
#include "message/input/CameraParameters.h"

namespace module {
namespace vision {

    using message::input::CameraParameters;
    using message::input::Sensors;

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

    using message::vision::ObjectClass;
    using message::vision::LookUpTable;
    using message::vision::ClassifiedImage;
    using message::vision::VisionObject;
    using message::vision::Goal;

    using message::support::Configuration;
    using message::support::FieldDescription;

    // TODO the system is too generous with adding segments above and below the goals and makes them too tall, stop it
    // TODO the system needs to throw out the kinematics and height based measurements when it cannot be sure it saw the tops and bottoms of the goals

    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

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
            measurement_distance_covariance_factor = config["measurement_distance_covariance_factor"].as<double>();
            measurement_bearing_variance = config["measurement_bearing_variance"].as<double>();
            measurement_elevation_variance = config["measurement_elevation_variance"].as<double>();
        });

        on<Trigger<ClassifiedImage<ObjectClass>>
         , With<CameraParameters>
         , With<LookUpTable>
         , Optional<With<FieldDescription>>
         , Single>().then("Goal Detector", [this] (std::shared_ptr<const ClassifiedImage<ObjectClass>> rawImage
                          , const CameraParameters& cam
                          , const LookUpTable& lut
                          , std::shared_ptr<const FieldDescription> field) {

            if (field == nullptr) {
                NUClear::log(__FILE__, ", ", __LINE__, ": FieldDescription Update: support::configuration::SoccerConfig module might not be installed.");
                throw std::runtime_error("FieldDescription Update: support::configuration::SoccerConfig module might not be installed");
            }
            const auto& image = *rawImage;
            auto& sensors = *image.sensors;
            // Our segments that may be a part of a goal
            std::vector<RansacGoalModel::GoalSegment> segments;
            auto goals = std::make_unique<std::vector<Goal>>();
            const double& GOAL_HEIGHT = field->dimensions.goal_crossbar_height;
            const double& GOAL_DIAMETER = field->dimensions.goalpost_diameter;

            // Get our goal segments
            auto hSegments = image.horizontalSegments.equal_range(ObjectClass::GOAL);
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on the other side
                if(it->second.subsample == 1 && it->second.previous && it->second.next) {
                    segments.push_back({ { double(it->second.start[0]), double(it->second.start[1]) }, { double(it->second.end[0]), double(it->second.end[1]) } });
                }
            }

            // Partition our segments so that they are split between above and below the horizon
            auto split = std::partition(std::begin(segments), std::end(segments), [image] (const RansacGoalModel::GoalSegment& segment) {
                // Is the midpoint above or below the horizon?
                return image.horizon.distanceToPoint(segment.left + segment.right / 2) > 0;
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

                    char c = lut(image.image->operator()(int(point[0]), int(point[1])));

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
                arma::vec2 p1 = midP1 - left.distanceToPoint(midP1) * arma::dot(left.normal, mid.normal) * mid.normal;
                arma::vec2 p2 = midP2 - left.distanceToPoint(midP2) * arma::dot(left.normal, mid.normal) * mid.normal;
                arma::vec2 p3 = midP1 - right.distanceToPoint(midP1) * arma::dot(right.normal, mid.normal) * mid.normal;
                arma::vec2 p4 = midP2 - right.distanceToPoint(midP2) * arma::dot(right.normal, mid.normal) * mid.normal;

                // Make a quad
                Goal g;
                g.side = Goal::Side::UNKNOWN;

                // Seperate tl and bl
                arma::vec2 tl = p1[1] > p2[1] ? p2 : p1;
                arma::vec2 bl = p1[1] > p2[1] ? p1 : p2;
                arma::vec2 tr = p3[1] > p4[1] ? p4 : p3;
                arma::vec2 br = p3[1] > p4[1] ? p3 : p4;

                g.quad.set(bl, tl, tr, br);

                g.sensors = image.sensors;
                g.classifiedImage = rawImage;
                goals->push_back(std::move(g));
            }

            // Throwout invalid quads
            for(auto it = goals->begin(); it != goals->end();) {

                auto& quad = it->quad;
                arma::vec2 lhs = arma::normalise(quad.getTopLeft() - quad.getBottomLeft());
                arma::vec2 rhs = arma::normalise(quad.getTopRight() - quad.getBottomRight());

                // Check if we are within the aspect ratio range
                bool valid = quad.aspectRatio() > MINIMUM_ASPECT_RATIO
                          && quad.aspectRatio() < MAXIMUM_ASPECT_RATIO
                // Check if we are close enough to the visual horizon
                          && (image.visualHorizonAtPoint(quad.getBottomLeft()[0]) < quad.getBottomLeft()[1] + VISUAL_HORIZON_BUFFER
                              || image.visualHorizonAtPoint(quad.getBottomRight()[0]) < quad.getBottomRight()[1] + VISUAL_HORIZON_BUFFER)
                // Check we finish above the kinematics horizon or or kinematics horizon is off the screen
                          && (image.horizon.y(quad.getTopLeft()[0]) > quad.getTopLeft()[1] || image.horizon.y(quad.getTopLeft()[0]) < 0)
                          && (image.horizon.y(quad.getTopRight()[0]) > quad.getTopRight()[1] || image.horizon.y(quad.getTopRight()[0]) < 0)
                // Check that our two goal lines are perpendicular with the horizon must use greater than rather then less than because of the cos
                          && std::abs(arma::dot(lhs, image.horizon.normal)) > MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE
                          && std::abs(arma::dot(rhs, image.horizon.normal)) > MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE
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
                for (auto b = std::next(a); b != goals->end();) {

                    if (a->quad.overlapsHorizontally(b->quad)) {
                        // Get outer lines.
                        arma::vec2 tl;
                        arma::vec2 tr;
                        arma::vec2 bl;
                        arma::vec2 br;

                        tl = { std::min(a->quad.getTopLeft()[0],     b->quad.getTopLeft()[0]),     std::min(a->quad.getTopLeft()[1],     b->quad.getTopLeft()[1]) };
                        tr = { std::max(a->quad.getTopRight()[0],    b->quad.getTopRight()[0]),    std::min(a->quad.getTopRight()[1],    b->quad.getTopRight()[1]) };
                        bl = { std::min(a->quad.getBottomLeft()[0],  b->quad.getBottomLeft()[0]),  std::max(a->quad.getBottomLeft()[1],  b->quad.getBottomLeft()[1]) };
                        br = { std::max(a->quad.getBottomRight()[0], b->quad.getBottomRight()[0]), std::max(a->quad.getBottomRight()[1], b->quad.getBottomRight()[1]) };

                        // Replace original two quads with the new one.
                        a->quad.set(bl, tl, tr, br);
                        b = goals->erase(b);
                    }
                    else {
                        b++;
                    }
                }
            }

            // Do the kinematics for the goals
            for(auto it = goals->begin(); it != goals->end(); ++it) {

                std::vector<VisionObject::Measurement> measurements;

                // Get the quad points in screen coords
                arma::vec2 tl = imageToScreen(it->quad.getTopLeft(), image.dimensions);
                arma::vec2 tr = imageToScreen(it->quad.getTopRight(), image.dimensions);
                arma::vec2 bl = imageToScreen(it->quad.getBottomLeft(), image.dimensions);
                arma::vec2 br = imageToScreen(it->quad.getBottomRight(), image.dimensions);

                arma::vec2 screenGoalCentre = (tl + tr + bl + br) * 0.25;




                // Projection rays ray
                arma::vec3 topRay = arma::normalise(arma::normalise(getCamFromScreen(tl, cam.focalLengthPixels))
                                                           + arma::normalise(getCamFromScreen(tr, cam.focalLengthPixels)));
                arma::vec3 baseRay = arma::normalise(arma::normalise(getCamFromScreen(bl, cam.focalLengthPixels))
                                                           + arma::normalise(getCamFromScreen(br, cam.focalLengthPixels)));
                arma::vec3 centreRay = arma::normalise((topRay + baseRay) * 0.5);

                // Measure the distance to the top of the goals
                Plane topOfGoalPlane({ 0, 0, 1 }, { 0, 0, GOAL_HEIGHT });
                arma::vec3 goalTopProj = projectCamToPlane(topRay, sensors.orientationCamToGround, topOfGoalPlane) - arma::vec({ 0, 0, GOAL_HEIGHT / 2 });
                double goalTopProjDist = arma::norm(goalTopProj);
                arma::mat goalTopProjDistCov = arma::diagmat(arma::vec({
                        goalTopProjDist * measurement_distance_covariance_factor,
                        measurement_bearing_variance,
                        measurement_elevation_variance }));
                measurements.push_back({cartesianToSpherical(goalTopProj), goalTopProjDistCov,
                                        arma::zeros<arma::vec>(3), arma::zeros<arma::mat>(3, 3)});

                // Measure the distance to the base of the goals
                Plane groundPlane({ 0, 0, 1 }, { 0, 0, 0 });
                arma::vec3 goalBaseProj = projectCamToPlane(baseRay, sensors.orientationCamToGround, groundPlane) + arma::vec({ 0, 0, GOAL_HEIGHT / 2 });
                double goalBaseProjDist = arma::norm(goalBaseProj);
                arma::mat goalBaseProjDistCov = arma::diagmat(arma::vec({
                        goalBaseProjDist * measurement_distance_covariance_factor,
                        measurement_bearing_variance,
                        measurement_elevation_variance }));
                measurements.push_back({cartesianToSpherical(goalBaseProj), goalBaseProjDistCov,
                                        arma::zeros<arma::vec>(3), arma::zeros<arma::mat>(3, 3)});


                // Measure the width based distance to the bottom
                double baseWidthDistance = widthBasedDistanceToCircle(GOAL_DIAMETER / 2, bl, br, cam.focalLengthPixels);
                arma::vec3 baseGoalWidth = baseWidthDistance * sensors.orientationCamToGround.submat(0,0,2,2) * baseRay + sensors.orientationCamToGround.submat(0,3,2,3) + arma::vec({ 0, 0, GOAL_HEIGHT / 2 });
                double baseGoalWidthDist = arma::norm(baseGoalWidth);
                arma::mat baseGoalWidthDistCov = arma::diagmat(arma::vec({
                        baseGoalWidthDist * measurement_distance_covariance_factor,
                        measurement_bearing_variance,
                        measurement_elevation_variance }));
                measurements.push_back({cartesianToSpherical(baseGoalWidth), baseGoalWidthDistCov,
                                        arma::zeros<arma::vec>(3), arma::zeros<arma::mat>(3, 3)});

                // Measure the width based distance to the top
                double topWidthDistance = widthBasedDistanceToCircle(GOAL_DIAMETER / 2, tl, tr, cam.focalLengthPixels);
                arma::vec3 topGoalWidth = topWidthDistance * sensors.orientationCamToGround.submat(0,0,2,2) * topRay + sensors.orientationCamToGround.submat(0,3,2,3) - arma::vec({ 0, 0, GOAL_HEIGHT / 2 });
                double topGoalWidthDist = arma::norm(topGoalWidth);
                arma::mat topGoalWidthDistCov = arma::diagmat(arma::vec({
                        topGoalWidthDist * measurement_distance_covariance_factor,
                        measurement_bearing_variance,
                        measurement_elevation_variance }));
                measurements.push_back({cartesianToSpherical(topGoalWidth), topGoalWidthDistCov,
                                        arma::zeros<arma::vec>(3), arma::zeros<arma::mat>(3, 3)});

                // Measure the height based distance
                double heightDistance = distanceToVerticalObject((tl + tr) * 0.5, (bl + br) * 0.5, GOAL_HEIGHT, sensors.orientationCamToGround(2,3), cam.focalLengthPixels);
                arma::vec3 goalHeight = heightDistance * sensors.orientationCamToGround.submat(0,0,2,2) * baseRay + sensors.orientationCamToGround.submat(0,3,2,3) + arma::vec({ 0, 0, GOAL_HEIGHT / 2 });
                double goalHeightDist = arma::norm(goalHeight);
                arma::mat goalHeightDistCov = arma::diagmat(arma::vec({
                        goalHeightDist * measurement_distance_covariance_factor,
                        measurement_bearing_variance,
                        measurement_elevation_variance }));
                measurements.push_back({cartesianToSpherical(goalHeight), goalHeightDistCov,
                                        arma::zeros<arma::vec>(3), arma::zeros<arma::mat>(3, 3)});


                // // Ignore invalid measurements:
                // arma::vec2 topMidGoal =(tl + tr) * 0.5;
                // arma::vec2 bottomMidGoal = (bl + br) * 0.5;

                // if (topMidGoal(1) < image.dimensions[1] / 2 - 10) {
                //     measurements.push_back({ cartesianToSpherical(goalTopProj), goalTopProjDistCov});
                //     if (bottomMidGoal(1) > -image.dimensions[1] / 2 + 10) {
                //         measurements.push_back({ cartesianToSpherical(goalHeight), goalHeightDistCov});
                //     }
                // }

                // if (bottomMidGoal(1) > -image.dimensions[1] / 2 + 10) {
                //     measurements.push_back({ cartesianToSpherical(goalBaseProj), goalBaseProjDistCov});
                // }


                // Add our variables
                it->measurements = measurements;
                it->sensors = image.sensors;
                /*log("Goal Measurements");
                for (auto m : it->measurements){
                    log(m.position);
                }*/

                // Angular positions from the camera
                it->screenAngular = arma::atan(cam.pixelsToTanThetaFactor % screenGoalCentre);
                arma::vec2 brAngular = arma::atan(cam.pixelsToTanThetaFactor % br);
                arma::vec2 trAngular = arma::atan(cam.pixelsToTanThetaFactor % tr);
                arma::vec2 blAngular = arma::atan(cam.pixelsToTanThetaFactor % bl);
                arma::vec2 tlAngular = arma::atan(cam.pixelsToTanThetaFactor % tl);
                Quad angularQuad(blAngular,tlAngular,trAngular,brAngular);
                it->angularSize = angularQuad.getSize();
            }

            // Do some extra throwouts for goals based on kinematics

            // Assign leftness and rightness to goals
            if (goals->size() == 2) {
                if (goals->at(0).quad.getCentre()(0) < goals->at(1).quad.getCentre()(0)) {
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

