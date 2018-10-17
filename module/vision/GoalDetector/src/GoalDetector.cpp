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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "GoalDetector.h"

#include <cmath>

#include "RansacGoalModel.h"

#include "extension/Configuration.h"

#include "message/input/CameraParameters.h"
#include "message/support/FieldDescription.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/LookUpTable.h"
#include "message/vision/VisionObjects.h"

#include "utility/math/coordinates.h"
#include "utility/math/geometry/Line.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/geometry/Quad.h"
#include "utility/math/ransac/NPartiteRansac.h"
#include "utility/math/vision.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/vision/ClassifiedImage.h"
#include "utility/vision/LookUpTable.h"
#include "utility/vision/Vision.h"


namespace module {
namespace vision {

    using extension::Configuration;

    using message::input::CameraParameters;
    using message::vision::ClassifiedImage;
    using message::vision::LookUpTable;
    using SegmentClass = message::vision::ClassifiedImage::SegmentClass::Value;
    using message::support::FieldDescription;
    using message::vision::Goal;

    using utility::math::coordinates::cartesianToSpherical;
    using Plane = utility::math::geometry::Plane<3>;
    using utility::math::geometry::Quad;
    using utility::math::ransac::NPartiteRansac;
    using utility::math::vision::distanceToVerticalObject;
    using utility::math::vision::getCamFromImage;
    using utility::math::vision::getCamFromScreen;
    using utility::math::vision::getImageFromCam;
    using utility::math::vision::getImageFromCamCts;
    using utility::math::vision::getParallaxAngle;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::vision::projectCamToPlane;
    using utility::math::vision::widthBasedDistanceToCircle;
    using utility::nusight::drawVisionLines;

    // TODO the system is too generous with adding segments above and below the goals and makes them too tall, stop it
    // TODO the system needs to throw out the kinematics and height based measurements when it cannot be sure it saw the
    // tops and bottoms of the goals

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
        , MAXIMUM_ANGLE_BETWEEN_SIDES(0.0)
        , MAXIMUM_VERTICAL_GOAL_PERSPECTIVE_ANGLE(0.0)
        , MEASUREMENT_LIMITS_LEFT(10)
        , MEASUREMENT_LIMITS_RIGHT(10)
        , MEASUREMENT_LIMITS_TOP(10)
        , MEASUREMENT_LIMITS_BASE(10) {


        // Trigger the same function when either update
        on<Configuration, Trigger<CameraParameters>>("GoalDetector.yaml")
            .then([this](const Configuration& config, const CameraParameters& cam) {
                MINIMUM_POINTS_FOR_CONSENSUS   = config["ransac"]["minimum_points_for_consensus"].as<uint>();
                CONSENSUS_ERROR_THRESHOLD      = config["ransac"]["consensus_error_threshold"].as<double>();
                MAXIMUM_ITERATIONS_PER_FITTING = config["ransac"]["maximum_iterations_per_fitting"].as<uint>();
                MAXIMUM_FITTED_MODELS          = config["ransac"]["maximum_fitted_models"].as<uint>();

                MINIMUM_ASPECT_RATIO = config["aspect_ratio_range"][0].as<double>();
                MAXIMUM_ASPECT_RATIO = config["aspect_ratio_range"][1].as<double>();

                arma::vec3 horizon_buffer_height = {1, 0, tan(config["visual_horizon_buffer"].as<double>())};
                // Max of 1 and y coordinate of cam space projection
                VISUAL_HORIZON_BUFFER = std::max(1, int(projectCamSpaceToScreen(horizon_buffer_height, cam)[1]));
                MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE =
                    std::cos(config["minimum_goal_horizon_angle"].as<double>() - M_PI_2);

                MAXIMUM_ANGLE_BETWEEN_SIDES = std::cos(config["maximum_angle_between_sides"].as<double>());
                MAXIMUM_VERTICAL_GOAL_PERSPECTIVE_ANGLE =
                    std::sin(-config["maximum_vertical_goal_perspective_angle"].as<double>());

                MEASUREMENT_LIMITS_LEFT  = config["measurement_limits"]["left"].as<uint>();
                MEASUREMENT_LIMITS_RIGHT = config["measurement_limits"]["right"].as<uint>();
                MEASUREMENT_LIMITS_TOP   = config["measurement_limits"]["top"].as<uint>();
                MEASUREMENT_LIMITS_BASE  = config["measurement_limits"]["base"].as<uint>();

                ANGULAR_WIDTH_DISAGREEMENT_THRESHOLD_VERTICAL =
                    config["angular_width_disagreement_threshold_vertical"].as<double>();
                ANGULAR_WIDTH_DISAGREEMENT_THRESHOLD_HORIZONTAL =
                    config["angular_width_disagreement_threshold_horizontal"].as<double>();

                VECTOR3_COVARIANCE = config["vector3_covariance"].as<arma::vec>();
                ANGLE_COVARIANCE   = config["angle_covariance"].as<arma::vec>();

                DEBUG_GOAL_THROWOUTS = config["debug_goal_throwouts"].as<bool>();
                DEBUG_GOAL_RANSAC    = config["debug_goal_ransac"].as<bool>();
            });

        on<Trigger<ClassifiedImage>, With<CameraParameters>, With<LookUpTable>, With<FieldDescription>, Single>().then(
            "Goal Detector",
            [this](std::shared_ptr<const ClassifiedImage> rawImage,
                   const CameraParameters& cam,
                   const LookUpTable& lut,
                   const FieldDescription& fd) {
                if (DEBUG_GOAL_RANSAC) log("Detecting goals");

                const auto& image = *rawImage;
                // Our segments that may be a part of a goal
                std::vector<RansacGoalModel::GoalSegment> segments;
                auto goals = std::make_unique<std::vector<Goal>>();
                // Get our goal segments
                for (const auto& segment : image.horizontalSegments) {

                    // We throw out points if they are:
                    // Less the full quality (subsampled)
                    // Do not have a transition on the other side
                    if ((segment.segmentClass == SegmentClass::GOAL) && (segment.subsample == 1)
                        && (segment.previous > -1) && (segment.next > -1)) {
                        segments.push_back({getCamFromScreen(imageToScreen(convert<int, 2>(segment.start),
                                                                           convert<uint, 2>(cam.imageSizePixels)),
                                                             cam),
                                            getCamFromScreen(imageToScreen(convert<int, 2>(segment.end),
                                                                           convert<uint, 2>(cam.imageSizePixels)),
                                                             cam)});
                    }
                }
                // Is the midpoint above or below the horizon?
                // Partition our segments so that they are split between above and below the horizon
                auto split = std::partition(
                    std::begin(segments), std::end(segments), [image](const RansacGoalModel::GoalSegment& segment) {
                        return arma::dot(convert<double, 3>(image.horizon_normal), (segment.left + segment.right) / 2)
                               > 0;
                    });

                // Make an array of our partitions
                std::array<std::vector<RansacGoalModel::GoalSegment>::iterator, RansacGoalModel::REQUIRED_POINTS + 1>
                    points = {segments.begin(), split, segments.end()};

                // Ransac for goals
                auto models = NPartiteRansac<RansacGoalModel>::fitModels(points,
                                                                         MINIMUM_POINTS_FOR_CONSENSUS,
                                                                         MAXIMUM_ITERATIONS_PER_FITTING,
                                                                         MAXIMUM_FITTED_MODELS,
                                                                         CONSENSUS_ERROR_THRESHOLD);

                if (DEBUG_GOAL_RANSAC) log("Ransac results ", models.size(), "from ", segments.size());
                std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>,
                            Eigen::aligned_allocator<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>>>
                    debug;
                // Look at our results
                for (auto& result : models) {
                    // Get our left, right and midlines
                    Plane& left  = result.model.leftPlane;
                    Plane& right = result.model.rightPlane;

                    if (DEBUG_GOAL_RANSAC) {

                        arma::vec3 debugLeftt = left.orthogonalProjection(arma::vec3({1, 0, 1}));
                        arma::vec3 debugLeftb = left.orthogonalProjection(arma::vec3({1, 0, -1}));

                        arma::vec3 debugRightt = right.orthogonalProjection(arma::vec3({1, 0, 1}));
                        arma::vec3 debugRightb = right.orthogonalProjection(arma::vec3({1, 0, -1}));
                        // DEBUG!
                        float N = 100;
                        for (int i = 0; i < 100; i++) {
                            float alpha              = i / N;
                            float alphaNext          = (i + 1) / N;
                            arma::vec3 debugRightPt1 = debugRightt * alpha + (1 - alpha) * debugRightb;
                            arma::vec3 debugRightPt2 = debugRightt * alphaNext + (1 - alphaNext) * debugRightb;

                            arma::vec3 debugLeftPt1 = debugLeftt * alpha + (1 - alpha) * debugLeftb;
                            arma::vec3 debugLeftPt2 = debugLeftt * alphaNext + (1 - alphaNext) * debugLeftb;

                            debug.push_back(std::make_tuple(convert<int, 2>(getImageFromCam(debugRightPt1, cam)),
                                                            convert<int, 2>(getImageFromCam(debugRightPt2, cam)),
                                                            Eigen::Vector4d(1, 0, 0, 1)));

                            debug.push_back(std::make_tuple(convert<int, 2>(getImageFromCam(debugLeftPt1, cam)),
                                                            convert<int, 2>(getImageFromCam(debugLeftPt2, cam)),
                                                            Eigen::Vector4d(0, 0, 1, 1)));
                        }
                    }

                    Plane mid;

                    // Normals in same direction
                    if (arma::dot(left.normal, right.normal) > 0) {
                        mid.normal = arma::normalise(right.normal + left.normal);
                    }
                    // Normals opposed
                    else {
                        mid.normal = arma::normalise(right.normal - left.normal);
                    }

                    // Find a point that should work to start searching down
                    arma::vec3 midpoint({0, 0, 0});
                    int i = 0;
                    for (auto& m : result) {
                        midpoint += m.left;
                        midpoint += m.right;
                        i += 2;
                    }
                    midpoint /= i;
                    midpoint = arma::normalise(midpoint);

                    // Work out which direction to go
                    arma::vec3 direction = arma::normalise(arma::cross(mid.normal, arma::vec3({1, 0, 0})));
                    // Rectify if pointing up
                    direction *= direction[2] < 0 ? 1 : -1;
                    // Move this direction
                    // TODO: configure
                    direction *= 0.01;


                    // Classify until we reach green
                    arma::vec3 basePoint({1, 0, 0});
                    int notWhiteLen        = 0;
                    arma::vec3 point       = arma::normalise(mid.orthogonalProjection(midpoint));
                    arma::ivec2 imagePoint = getImageFromCam(point, cam);
                    float color_intensity  = 0;
                    while ((imagePoint[0] < int(image.dimensions[0])) && (imagePoint[0] > 0)
                           && (imagePoint[1] < int(image.dimensions[1]))) {

                        char c = static_cast<char>(utility::vision::getPixelColour(
                            lut,
                            utility::vision::getPixel(imagePoint[0],
                                                      imagePoint[1],
                                                      image.image->dimensions[0],
                                                      image.image->dimensions[1],
                                                      image.image->data,
                                                      static_cast<utility::vision::FOURCC>(image.image->format))));

                        if (c != 'y') {
                            ++notWhiteLen;
                            if (notWhiteLen > 4) {
                                basePoint = point;
                                break;
                            }
                        }
                        else if (c == 'g') {
                            basePoint = point;
                            break;
                        }
                        else if (c == 'y') {
                            notWhiteLen = 0;
                        }
                        if (DEBUG_GOAL_RANSAC) {
                            debug.push_back(std::make_tuple(convert<int, 2>(imagePoint),
                                                            convert<int, 2>(imagePoint + arma::ivec({1, 1})),
                                                            Eigen::Vector4d(color_intensity, 0, color_intensity, 1)));
                            color_intensity = std::fmin(1, color_intensity + 0.5);
                        }
                        point += direction;
                        imagePoint = getImageFromCam(point, cam);
                    }


                    arma::running_stat<double> stat;

                    float min = 1, max = -1;
                    arma::vec3 minPt, maxPt;
                    // Look through our segments to find endpoints
                    arma::vec3 horizon_normal = convert<double, 3>(image.horizon_normal);
                    for (auto& segment : result) {
                        // Project left and right onto midpoint keep top and bottom
                        auto leftDot  = arma::dot(horizon_normal, segment.left);
                        auto rightDot = arma::dot(horizon_normal, segment.right);

                        if (leftDot < min) {
                            min   = leftDot;
                            minPt = segment.left;
                        }
                        if (rightDot < min) {
                            min   = rightDot;
                            minPt = segment.right;
                        }
                        if (leftDot > max) {
                            max   = leftDot;
                            maxPt = segment.left;
                        }
                        if (rightDot > max) {
                            max   = rightDot;
                            maxPt = segment.right;
                        }
                    }

                    // Get our endpoints from the min and max points on the line
                    arma::vec3 midTop  = arma::normalise(mid.orthogonalProjection(maxPt));
                    arma::vec3 midBase = arma::normalise(mid.orthogonalProjection(basePoint));

                    // Make a quad
                    Goal goal;
                    goal.visObject.sensors = image.sensors;
                    goal.side              = Goal::Side::UNKNOWN_SIDE;

                    // Project those points outward onto the quad
                    arma::vec3 horizonScreenDir = arma::normalise(
                        arma::cross(convert<double, 3, 1>(image.horizon_normal), arma::vec3({1, 0, 0})));
                    arma::vec3 tl = left.directionalProjection(midTop, horizonScreenDir);
                    arma::vec3 tr = right.directionalProjection(midTop, horizonScreenDir);
                    arma::vec3 bl = left.directionalProjection(midBase, horizonScreenDir);
                    arma::vec3 br = right.directionalProjection(midBase, horizonScreenDir);

                    goal.frustum.bl = convert<double, 3>(bl);
                    goal.frustum.tl = convert<double, 3>(tl);
                    goal.frustum.tr = convert<double, 3>(tr);
                    goal.frustum.br = convert<double, 3>(br);

                    // In image coords
                    goal.quad.bl = convert<double, 2>(getImageFromCamCts(bl, cam));
                    goal.quad.tl = convert<double, 2>(getImageFromCamCts(tl, cam));
                    goal.quad.tr = convert<double, 2>(getImageFromCamCts(tr, cam));
                    goal.quad.br = convert<double, 2>(getImageFromCamCts(br, cam));


                    goals->push_back(std::move(goal));
                }
                if (DEBUG_GOAL_RANSAC) {
                    emit(drawVisionLines(debug));
                }


                // Throwout invalid quads
                for (auto it = goals->begin(); it != goals->end();) {

                    arma::vec3 cbl = convert<double, 3>(it->frustum.bl);
                    arma::vec3 ctl = convert<double, 3>(it->frustum.tl);
                    arma::vec3 ctr = convert<double, 3>(it->frustum.tr);
                    arma::vec3 cbr = convert<double, 3>(it->frustum.br);

                    float leftAngle   = std::acos(arma::norm_dot(cbl, ctl));
                    float rightAngle  = std::acos(arma::norm_dot(cbr, ctr));
                    float topAngle    = std::acos(arma::norm_dot(ctr, ctl));
                    float bottomAngle = std::acos(arma::norm_dot(cbr, cbl));

                    // Get the centre ray of goal by averaging points
                    arma::vec3 goalCentreRay = cbl + ctl + ctr + cbr;
                    goalCentreRay /= 4;

                    // float dAngleVertical   = std::fabs(leftAngle - rightAngle);
                    // float dAngleHorizontal = std::fabs(topAngle - bottomAngle);
                    it->visObject.screenAngular = convert<double, 2>(cartesianToSpherical(goalCentreRay).rows(1, 2));

                    float vertAngle           = (leftAngle + rightAngle) / 2;
                    float horAngle            = (topAngle + bottomAngle) / 2;
                    it->visObject.angularSize = Eigen::Vector2d(horAngle, vertAngle);

                    bool valid        = vertAngle > 0;
                    float aspectRatio = vertAngle / horAngle;

                    // Check if we are within the aspect ratio range

                    bool aspectRatioGood = aspectRatio > MINIMUM_ASPECT_RATIO && aspectRatio < MAXIMUM_ASPECT_RATIO;

                    bool shapeConsistent =
                        std::fabs(leftAngle - rightAngle) < ANGULAR_WIDTH_DISAGREEMENT_THRESHOLD_VERTICAL
                        && std::fabs(topAngle - bottomAngle) < ANGULAR_WIDTH_DISAGREEMENT_THRESHOLD_HORIZONTAL;

                    // Check if we are close enough to the visual horizon

                    bool closeToVisualHorizon = utility::vision::visualHorizonAtPoint(image, it->quad.bl[0])
                                                    < it->quad.bl[1] + VISUAL_HORIZON_BUFFER
                                                || utility::vision::visualHorizonAtPoint(image, it->quad.br[0])
                                                       < it->quad.br[1] + VISUAL_HORIZON_BUFFER;

                    // horizon is off the screen

                    bool aboveHorizon = (arma::dot(convert<double, 3>(image.horizon_normal), ctr) > 0)
                                        || (arma::dot(convert<double, 3>(image.horizon_normal), ctl) > 0);

                    // Check that our two goal lines are perpendicular with the horizon must use greater than rather
                    // then less than because of the cos
                    bool goalsOrthogonalToHorizon =
                        std::abs(arma::norm_dot(arma::cross(cbr, ctr), convert<double, 3>(image.horizon_normal)))
                            < MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE
                        && std::abs(arma::norm_dot(arma::cross(cbl, ctl), convert<double, 3>(image.horizon_normal)))
                               < MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE;

                    // Check that our two goal lines are approximatly parallel
                    bool goalsParallel =
                        std::abs(arma::dot(arma::cross(cbr, ctr), arma::cross(cbl, ctl))) < MAXIMUM_ANGLE_BETWEEN_SIDES;

                    valid = valid && aspectRatioGood && shapeConsistent && closeToVisualHorizon && aboveHorizon
                            && goalsOrthogonalToHorizon && goalsParallel;

                    if (!valid) {
                        if (DEBUG_GOAL_THROWOUTS) {
                            log("GOAL DISCARDED");
                            if (!aspectRatioGood) {
                                log("NOT TRUE : aspectRatioGood");
                                log("aspectRatio = ", aspectRatio);
                            }
                            if (!shapeConsistent) {
                                log("NOT TRUE : shapeConsistent");
                                log("vertical shape =", leftAngle, rightAngle);
                                log("horizontal shape =", topAngle, bottomAngle);
                            }
                            if (!closeToVisualHorizon) {
                                log("NOT TRUE : closeToVisualHorizon");
                                log("left base point",
                                    it->quad.bl[1],
                                    " !> hor - buffer (= ",
                                    utility::vision::visualHorizonAtPoint(image, it->quad.bl[0])
                                        - VISUAL_HORIZON_BUFFER,
                                    ")");
                                log("right base point",
                                    it->quad.br[1],
                                    " !> hor - buffer (= ",
                                    utility::vision::visualHorizonAtPoint(image, it->quad.br[0])
                                        - VISUAL_HORIZON_BUFFER,
                                    ")");
                            }
                            if (!aboveHorizon) {
                                log("NOT TRUE : aboveHorizon");
                                log("horizon_normal = ", convert<double, 3>(image.horizon_normal).t());
                                log("ctr = ", ctr);
                                log("ctl = ", ctl);
                            }
                            if (!goalsOrthogonalToHorizon) {
                                log("NOT TRUE : goalsOrthogonalToHorizon");
                                log("angle right = ",
                                    std::abs(arma::norm_dot(arma::cross(cbr, ctr),
                                                            convert<double, 3>(image.horizon_normal))));
                                log("angle left = ",
                                    std::abs(arma::norm_dot(arma::cross(cbl, ctl),
                                                            convert<double, 3>(image.horizon_normal))));
                            }
                            if (!goalsParallel) {
                                log("NOT TRUE : goalsParallel");
                                log("angle between sides = ",
                                    std::abs(arma::dot(arma::cross(cbr, ctr), arma::cross(cbl, ctl))));
                            }
                        }
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

                            tl = {std::min(aquad.getTopLeft()[0], bquad.getTopLeft()[0]),
                                  std::min(aquad.getTopLeft()[1], bquad.getTopLeft()[1])};
                            tr = {std::max(aquad.getTopRight()[0], bquad.getTopRight()[0]),
                                  std::min(aquad.getTopRight()[1], bquad.getTopRight()[1])};
                            bl = {std::min(aquad.getBottomLeft()[0], bquad.getBottomLeft()[0]),
                                  std::max(aquad.getBottomLeft()[1], bquad.getBottomLeft()[1])};
                            br = {std::max(aquad.getBottomRight()[0], bquad.getBottomRight()[0]),
                                  std::max(aquad.getBottomRight()[1], bquad.getBottomRight()[1])};

                            // Replace original two quads with the new one.
                            aquad.set(bl, tl, tr, br);
                            a->quad.bl = convert<double, 2>(bl);
                            a->quad.tl = convert<double, 2>(tl);
                            a->quad.tr = convert<double, 2>(tr);
                            a->quad.br = convert<double, 2>(br);
                            b          = goals->erase(b);
                        }
                        else {
                            b++;
                        }
                    }
                }

                // Store our measurements
                for (auto it = goals->begin(); it != goals->end(); ++it) {
                    utility::math::geometry::Quad quad(convert<double, 2>(it->quad.bl),
                                                       convert<double, 2>(it->quad.tl),
                                                       convert<double, 2>(it->quad.tr),
                                                       convert<double, 2>(it->quad.br));

                    // Get the quad points in screen coords
                    arma::vec2 tl = imageToScreen(quad.getTopLeft(), convert<uint, 2>(image.dimensions));
                    arma::vec2 tr = imageToScreen(quad.getTopRight(), convert<uint, 2>(image.dimensions));
                    arma::vec2 bl = imageToScreen(quad.getBottomLeft(), convert<uint, 2>(image.dimensions));
                    arma::vec2 br = imageToScreen(quad.getBottomRight(), convert<uint, 2>(image.dimensions));
                    arma::vec2 screenGoalCentre = (tl + tr + bl + br) * 0.25;

                    // Get vectors for TL TR BL BR;
                    arma::vec3 ctl       = convert<double, 3>(it->frustum.tl);
                    arma::vec3 ctr       = convert<double, 3>(it->frustum.tr);
                    arma::vec3 cbl       = convert<double, 3>(it->frustum.bl);
                    arma::vec3 cbr       = convert<double, 3>(it->frustum.br);
                    arma::vec3 rGCc_norm = arma::normalise((cbl + cbr) * 0.5);  // vector to bottom centre of goal post

                    // Get our four normals for each edge
                    // BL TL cross product gives left side
                    auto left                   = convert<double, 3>(arma::normalise(arma::cross(cbl, ctl)));
                    Eigen::Matrix3d left_vecCov = convert<double, 3, 3>(arma::diagmat(VECTOR3_COVARIANCE));
                    Eigen::Vector2d left_Angles(std::atan2(left[1], left[0]),
                                                std::atan2(left[2], std::sqrt(left[0] * left[0] + left[1] * left[1])));
                    Eigen::Matrix2d left_AngCov = convert<double, 2, 2>(arma::diagmat(ANGLE_COVARIANCE));

                    it->measurement.push_back(Goal::Measurement(
                        Goal::MeasurementType::LEFT_NORMAL, left, left_vecCov, left_Angles, left_AngCov));

                    // TR BR cross product gives right side
                    auto right = convert<double, 3>(arma::normalise(arma::cross(ctr, cbr)));

                    Eigen::Matrix3d right_vecCov = convert<double, 3, 3>(arma::diagmat(VECTOR3_COVARIANCE));
                    Eigen::Vector2d right_Angles(
                        std::atan2(right[1], right[0]),
                        std::atan2(right[2], std::sqrt(right[0] * right[0] + right[1] * right[1])));
                    Eigen::Matrix2d right_AngCov = convert<double, 2, 2>(arma::diagmat(ANGLE_COVARIANCE));


                    it->measurement.push_back(Goal::Measurement(
                        Goal::MeasurementType::RIGHT_NORMAL, right, right_vecCov, right_Angles, right_AngCov));

                    // Check that the bottom of the goal is not too close to the edges of the screen
                    if (std::min(quad.getBottomRight()[0], quad.getBottomLeft()[0]) > MEASUREMENT_LIMITS_LEFT
                        && std::min(quad.getBottomRight()[1], quad.getBottomLeft()[1]) > MEASUREMENT_LIMITS_TOP
                        && cam.imageSizePixels[0] - std::max(quad.getBottomRight()[0], quad.getBottomLeft()[0])
                               > MEASUREMENT_LIMITS_TOP
                        && cam.imageSizePixels[1] - std::max(quad.getBottomRight()[1], quad.getBottomLeft()[1])
                               > MEASUREMENT_LIMITS_BASE) {

                        // BR BL cross product gives the bottom side
                        auto bottom = convert<double, 3>(arma::normalise(arma::cross(cbr, cbl)));
                        it->measurement.push_back(Goal::Measurement(Goal::MeasurementType::BASE_NORMAL, bottom));

                        // Vector to the bottom centre average top and bottom distances
                        float distance_top = utility::math::vision::distanceToEquidistantCamPoints(
                            fd.dimensions.goalpost_width, ctl, ctr);
                        float distance_bottom = utility::math::vision::distanceToEquidistantCamPoints(
                            fd.dimensions.goalpost_width, cbl, cbr);
                        float distance = (distance_top + distance_bottom) / 2;

                        auto rGCc_sphr = convert<double, 3>(cartesianToSpherical(distance * rGCc_norm));
                        arma::vec3 covariance_amplifier({distance, 1, 1});
                        Eigen::Matrix3d rGCc_cov =
                            convert<double, 3, 3>(arma::diagmat(VECTOR3_COVARIANCE % covariance_amplifier));

                        if (std::isfinite(rGCc_sphr[0]) && std::isfinite(rGCc_sphr[1]) && std::isfinite(rGCc_sphr[2])) {
                            it->measurement.push_back(
                                Goal::Measurement(Goal::MeasurementType::CENTRE, rGCc_sphr, rGCc_cov));
                        }
                    }

                    // Check that the points are not too close to the edges of the screen
                    if (std::min(ctr[0], ctl[0]) > MEASUREMENT_LIMITS_LEFT
                        && std::min(ctr[1], ctl[1]) > MEASUREMENT_LIMITS_TOP
                        && cam.imageSizePixels[0] - std::max(ctr[0], ctl[0]) < MEASUREMENT_LIMITS_TOP
                        && cam.imageSizePixels[1] - std::max(ctr[1], ctl[1]) < MEASUREMENT_LIMITS_BASE) {

                        // TL TR cross product gives the top side
                        auto top = convert<double, 3>(arma::normalise(arma::cross(ctl, ctr)));
                        it->measurement.push_back(Goal::Measurement(Goal::MeasurementType::TOP_NORMAL, top));
                    }

                    // Add classified image corresponding to this message
                    it->visObject.classifiedImage = const_cast<ClassifiedImage*>(rawImage.get())->shared_from_this();
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
                    }
                    else {
                        goals->at(0).side = Goal::Side::RIGHT;
                        goals->at(1).side = Goal::Side::LEFT;
                    }
                }

                emit(std::move(goals));
            });
    }
}  // namespace vision
}  // namespace module
