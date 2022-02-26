/*
 * This file is part of the NUbots Codebase.
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

#include "HeadBehaviourSoccer.hpp"

#include <string>
#include <utility>

#include "extension/Configuration.hpp"

#include "message/localisation/Field.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/HeadCommand.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Goal.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/motion/InverseKinematics.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::behaviour::skills {

    using extension::Configuration;

    using message::behaviour::SoccerObjectPriority;
    using SearchType = message::behaviour::SoccerObjectPriority::SearchType;
    using message::input::Image;
    using message::input::Sensors;
    using LocBall = message::localisation::Ball;
    using message::motion::ExecuteGetup;
    using message::motion::HeadCommand;
    using message::motion::KillGetup;
    using message::motion::KinematicsModel;
    using message::vision::Ball;
    using message::vision::Balls;
    using message::vision::Goal;
    using message::vision::Goals;

    using utility::input::ServoID;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::math::geometry::Quad;
    using utility::motion::kinematics::calculateHeadJoints;
    using utility::support::Expression;

    inline Eigen::Vector2d screenAngularFromObjectDirection(const Eigen::Vector3d& v) {
        return {std::atan2(v.y(), v.x()), std::atan2(v.z(), v.x())};
    }

    inline Eigen::Vector3d objectDirectionFromScreenAngular(const Eigen::Vector2d& screen_angular) {
        if (std::fmod(std::fabs(screen_angular.x()), M_PI) == M_PI_2
            || std::fmod(std::fabs(screen_angular.y()), M_PI) == M_PI_2) {
            return {0, 0, 0};
        }
        double tanTheta        = std::tan(screen_angular.x());
        double tanPhi          = std::tan(screen_angular.y());
        double x               = 0;
        double y               = 0;
        double z               = 0;
        double denominator_sqr = 1 + tanTheta * tanTheta + tanPhi * tanPhi;
        // Assume facing forward st x>0 (which is fine for screen angular)
        x = 1 / std::sqrt(denominator_sqr);
        y = x * tanTheta;
        z = x * tanPhi;

        return {x, y, z};
    }

    HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("HeadBehaviourSoccer.yaml")
            .then("Head Behaviour Soccer Config", [this](const Configuration& config) {
                log_level = config["log_level"].as<NUClear::LogLevel>();

                lastPlanUpdate     = NUClear::clock::now();
                timeLastObjectSeen = NUClear::clock::now();

                // Config HeadBehaviourSoccer.yaml
                fractional_view_padding = config["fractional_view_padding"].as<double>();

                search_timeout_ms = config["search_timeout_ms"].as<float>();

                fractional_angular_update_threshold = config["fractional_angular_update_threshold"].as<float>();

                headSearcher.setSwitchTime(config["fixation_time_ms"].as<float>());

                oscillate_search = config["oscillate_search"].as<bool>();

                // Note that these are actually modified later and are hence camelcase
                ballPriority = config["initial"]["priority"]["ball"].as<int>();
                goalPriority = config["initial"]["priority"]["goal"].as<int>();

                replan_angle_threshold = config["replan_angle_threshold"].as<float>();

                pitch_plan_threshold = config["pitch_plan_threshold"].as<float>() * M_PI / 180.0f;
                pitch_plan_value     = config["pitch_plan_value"].as<float>() * M_PI / 180.0f;

                // Load searches:
                for (const auto& search : config["searches"].config) {
                    SearchType s(search["search_type"].as<std::string>());
                    searches[s] = std::vector<Eigen::Vector2d>();
                    for (const auto& p : search["points"]) {
                        searches[s].push_back(p.as<Expression>());
                    }
                }
            });


        // TODO(BehaviourTeam): remove this horrible code
        // Check to see if we are currently in the process of getting up.
        on<Trigger<ExecuteGetup>>().then([this] { isGettingUp = true; });

        // Check to see if we have finished getting up.
        on<Trigger<KillGetup>>().then([this] { isGettingUp = false; });


        on<Trigger<SoccerObjectPriority>, Sync<HeadBehaviourSoccer>>().then("Head Behaviour Soccer - Set priorities",
                                                                            [this](const SoccerObjectPriority& p) {
                                                                                ballPriority = p.ball;
                                                                                goalPriority = p.goal;
                                                                                searchType   = p.search_type;
                                                                            });

        auto initialPriority  = std::make_unique<SoccerObjectPriority>();
        initialPriority->ball = 1;
        emit(initialPriority);


        on<Trigger<Sensors>,
           Optional<With<Balls>>,
           Optional<With<Goals>>,
           Optional<With<LocBall>>,
           With<KinematicsModel>,
           With<Image>,
           Single,
           Sync<HeadBehaviourSoccer>>()
            .then("Head Behaviour Main Loop",
                  [this](const Sensors& sensors,
                         const std::shared_ptr<const Balls>& vballs,
                         const std::shared_ptr<const Goals>& vgoals,
                         const std::shared_ptr<const LocBall>& locBall,
                         const KinematicsModel& kinematicsModel,
                         const Image& image) {
                      max_yaw   = kinematicsModel.head.MAX_YAW;
                      min_yaw   = kinematicsModel.head.MIN_YAW;
                      max_pitch = kinematicsModel.head.MAX_PITCH;
                      min_pitch = kinematicsModel.head.MIN_PITCH;

                      // std::cout << "Seen: Balls: " <<
                      // ((vballs != nullptr) ? std::to_string(int(vballs->size())) : std::string("null")) <<
                      // "Goals: " <<
                      // ((vgoals != nullptr) ? std::to_string(int(vgoals->size())) : std::string("null")) <<
                      // std::endl;

                      if (locBall) {
                          locBallReceived = true;
                          lastLocBall     = *locBall;
                      }
                      const auto& now = NUClear::clock::now();

                      bool objectsMissing = false;

                      // Get the list of objects which are currently visible
                      Balls ballFixationObjects = getFixationObjects(vballs, objectsMissing);
                      Goals goalFixationObjects = getFixationObjects(vgoals, objectsMissing);

                      // Determine state transition variables
                      const bool lost = ((ballFixationObjects.balls.empty()) && (goalFixationObjects.goals.empty()));
                      // Do we need to update our plan?
                      bool updatePlan =
                          !isGettingUp && ((lastBallPriority != ballPriority) || (lastGoalPriority != goalPriority));
                      // Has it been a long time since we have seen anything of interest?
                      const bool searchTimedOut =
                          std::chrono::duration_cast<std::chrono::milliseconds>(now - timeLastObjectSeen).count()
                          > search_timeout_ms;
                      // Did the object move in IMUspace?
                      bool objectMoved = false;

                      const bool ballMaxPriority = (ballPriority == std::max(ballPriority, goalPriority));

                      // log("updatePlan", updatePlan);
                      // log("lost", lost);
                      // log("isGettingUp", isGettingUp);
                      // log("searchType", int(searchType));
                      // log("headSearcher.size()", headSearcher.size());

                      // State execution

                      // Get robot heat to body transform
                      Eigen::Matrix3d orientation;
                      Eigen::Matrix3d headToBodyRotation;
                      if (!lost) {
                          // We need to transform our view points to orientation space
                          if (ballMaxPriority) {
                              Eigen::Affine3d Htc(sensors.Htw * ballFixationObjects.Hcw.inverse());
                              headToBodyRotation = Htc.rotation();
                              orientation        = Eigen::Affine3d(ballFixationObjects.Hcw).rotation().transpose();
                          }
                          else {
                              Eigen::Affine3d Htc(sensors.Htw * goalFixationObjects.Hcw.inverse());
                              headToBodyRotation = Htc.rotation();
                              orientation        = Eigen::Affine3d(goalFixationObjects.Hcw).rotation().transpose();
                          }
                      }
                      else {
                          Eigen::Affine3d Htc(sensors.Htx[ServoID::HEAD_PITCH]);
                          headToBodyRotation = Htc.rotation();
                          orientation        = Eigen::Affine3d(sensors.Htw).rotation().transpose();
                      }
                      Eigen::Matrix3d headToIMUSpace = orientation * headToBodyRotation;

                      // If objects visible, check current centroid to see if it moved
                      if (!lost) {
                          Eigen::Vector2d currentCentroid(Eigen::Vector2d::Zero());
                          if (ballMaxPriority) {
                              for (auto& ob : ballFixationObjects.balls) {
                                  currentCentroid +=
                                      (ob.screen_angular.cast<double>() / double(ballFixationObjects.balls.size()));
                              }
                          }
                          else {
                              for (auto& ob : goalFixationObjects.goals) {
                                  currentCentroid +=
                                      ob.screen_angular.cast<double>() / double(goalFixationObjects.goals.size());
                              }
                          }
                          Eigen::Vector2d currentCentroid_world = getIMUSpaceDirection(currentCentroid, headToIMUSpace);
                          // If our objects have moved, we need to replan
                          if ((currentCentroid_world - lastCentroid).norm()
                              >= fractional_angular_update_threshold * image.lens.fov / 2.0) {
                              objectMoved  = true;
                              lastCentroid = currentCentroid_world;
                          }
                      }

                      // State Transitions
                      if (!isGettingUp) {
                          switch (state) {
                              case FIXATION:
                                  if (lost) {
                                      state = WAIT;
                                  }
                                  else if (objectMoved) {
                                      updatePlan = true;
                                  }
                                  break;
                              case WAIT:
                                  if (!lost) {
                                      state      = FIXATION;
                                      updatePlan = true;
                                  }
                                  else if (searchTimedOut) {
                                      state      = SEARCH;
                                      updatePlan = true;
                                  }
                                  break;
                              case SEARCH:
                                  if (!lost) {
                                      state      = FIXATION;
                                      updatePlan = true;
                                  }
                                  break;
                          }
                      }

                      // If we arent getting up, then we can update the plan if necessary
                      if (updatePlan) {
                          if (lost) {
                              Rtw = Eigen::Affine3d(sensors.Htw).rotation();
                          }
                          if (ballMaxPriority) {
                              updateHeadPlan(kinematicsModel,
                                             ballFixationObjects,
                                             objectsMissing,
                                             sensors,
                                             headToIMUSpace,
                                             image.lens);
                          }

                          else {
                              updateHeadPlan(kinematicsModel,
                                             goalFixationObjects,
                                             objectsMissing,
                                             sensors,
                                             headToIMUSpace,
                                             image.lens);
                          }
                      }

                      // Update searcher
                      headSearcher.update(oscillate_search);
                      // Emit new result if possible
                      if (headSearcher.newGoal()) {
                          // Emit result
                          Eigen::Vector2d direction            = headSearcher.getState();
                          std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                          command->yaw                         = direction[0];
                          command->pitch                       = direction[1];
                          command->robot_space                 = (state == SEARCH);
                          // log("head angles robot space :", command->robot_space);
                          emit(std::move(command));
                      }

                      lastGoalPriority = goalPriority;
                      lastBallPriority = ballPriority;
                  });
    }

    Balls HeadBehaviourSoccer::getFixationObjects(const std::shared_ptr<const Balls>& vballs, bool& search) {

        auto now = NUClear::clock::now();
        Balls fixationObjects;

        int maxPriority = std::max(std::max(ballPriority, goalPriority), 0);
        if (ballPriority == goalPriority) {
            log<NUClear::WARN>("HeadBehaviourSoccer - Multiple object searching currently not supported properly.");
        }

        // Get balls
        if (ballPriority == maxPriority) {
            if (vballs && !vballs->balls.empty()) {
                // Fixate on ball
                timeLastObjectSeen = now;
                const auto& ball   = vballs->balls.at(0);
                fixationObjects.balls.push_back(ball);
            }
            else {
                search = true;
            }
        }

        return fixationObjects;
    }

    Goals HeadBehaviourSoccer::getFixationObjects(const std::shared_ptr<const Goals>& vgoals, bool& search) {

        auto now = NUClear::clock::now();
        Goals fixationObjects;

        int maxPriority = std::max(std::max(ballPriority, goalPriority), 0);
        if (ballPriority == goalPriority) {
            log<NUClear::WARN>("HeadBehaviourSoccer - Multiple object searching currently not supported properly.");
        }

        // TODO(BehaviourTeam): make this a loop over a list of objects or something
        // Get goals
        if (goalPriority == maxPriority) {
            if (vgoals && !vgoals->goals.empty()) {
                // Fixate on goals and lines and other landmarks
                timeLastObjectSeen = now;
                std::set<Goal::Side> visiblePosts;
                // TODO(BehaviourTeam): treat goals as one object
                Goals goals;
                for (const auto& goal : vgoals->goals) {
                    visiblePosts.insert(goal.side);
                    goals.goals.push_back(goal);
                }
                fixationObjects.goals.push_back(combineVisionObjects(goals));
                search = (visiblePosts.find(Goal::Side::LEFT) == visiblePosts.end() ||  // If left post not visible
                          visiblePosts.find(Goal::Side::RIGHT) == visiblePosts.end());  // or right post not
                                                                                        // visible, then we need to
                                                                                        // search for the other goal
                                                                                        // post
            }
            else {
                search = true;
            }
        }

        return fixationObjects;
    }


    void HeadBehaviourSoccer::updateHeadPlan(const KinematicsModel& kinematicsModel,
                                             const Balls& fixationObjects,
                                             const bool& search,
                                             const Sensors& sensors,
                                             const Eigen::Matrix3d& headToIMUSpace,
                                             const Image::Lens& lens) {
        std::vector<Eigen::Vector2d> fixationPoints;
        std::vector<Eigen::Vector2d> fixationSizes;
        Eigen::Vector2d currentPos = {sensors.servo[ServoID::HEAD_YAW].present_position,
                                      sensors.servo[ServoID::HEAD_PITCH].present_position};

        for (const auto& ball : fixationObjects.balls) {
            // Should be vec2 (yaw,pitch)
            fixationPoints.emplace_back(ball.screen_angular.x(), ball.screen_angular.y());
            fixationSizes.emplace_back(ball.angular_size.x(), ball.angular_size.y());
        }

        // If there are objects to find
        if (search) {
            fixationPoints = getSearchPoints(kinematicsModel, fixationObjects, searchType, sensors, lens);
        }

        if (fixationPoints.empty()) {
            log("FOUND NO POINTS TO LOOK AT! - ARE THE SEARCHES PROPERLY CONFIGURED IN "
                "HEADBEHAVIOURSOCCER.YAML?");
        }

        // Transform to IMU space including compensation for current head pose
        if (!search) {
            for (auto& p : fixationPoints) {
                p = getIMUSpaceDirection(p, headToIMUSpace);
            }
            currentPos = getIMUSpaceDirection(currentPos, headToIMUSpace);
        }

        headSearcher.replaceSearchPoints(fixationPoints, currentPos);
    }

    void HeadBehaviourSoccer::updateHeadPlan(const KinematicsModel& kinematicsModel,
                                             const Goals& fixationObjects,
                                             const bool& search,
                                             const Sensors& sensors,
                                             const Eigen::Matrix3d& headToIMUSpace,
                                             const Image::Lens& lens) {
        std::vector<Eigen::Vector2d> fixationPoints;
        std::vector<Eigen::Vector2d> fixationSizes;

        Eigen::Vector2d currentPos(sensors.servo[ServoID::HEAD_YAW].present_position,
                                   sensors.servo[ServoID::HEAD_PITCH].present_position);

        for (const auto& goal : fixationObjects.goals) {
            // Should be vec2 (yaw,pitch)
            fixationPoints.emplace_back(goal.screen_angular.x(), goal.screen_angular.y());
            fixationSizes.emplace_back(goal.angular_size.x(), goal.angular_size.y());
        }

        // If there are objects to find
        if (search) {
            fixationPoints = getSearchPoints(kinematicsModel, fixationObjects, searchType, sensors, lens);
        }

        if (fixationPoints.empty()) {
            log("FOUND NO POINTS TO LOOK AT! - ARE THE SEARCHES PROPERLY CONFIGURED IN "
                "HEADBEHAVIOURSOCCER.YAML?");
        }

        // Transform to IMU space including compensation for current head pose
        if (!search) {
            for (auto& p : fixationPoints) {
                p = getIMUSpaceDirection(p, headToIMUSpace);
            }
            currentPos = getIMUSpaceDirection(currentPos, headToIMUSpace);
        }

        headSearcher.replaceSearchPoints(fixationPoints, currentPos);
    }

    Eigen::Vector2d HeadBehaviourSoccer::getIMUSpaceDirection(const Eigen::Vector2d& screenAngles,
                                                              const Eigen::Matrix3d& headToIMUSpace) {

        // Eigen::Vector3d lookVectorFromHead = objectDirectionFromScreenAngular(screenAngles);
        // This is an approximation relying on the robots small FOV
        Eigen::Vector3d lookVectorFromHead =
            sphericalToCartesian(Eigen::Vector3d(1.0, screenAngles.x(), screenAngles.y()));
        // Remove pitch from matrix if we are adjusting search points

        // Rotate target angles to World space
        Eigen::Vector3d lookVector = headToIMUSpace * lookVectorFromHead;
        // Compute inverse kinematics for head direction angles
        std::vector<std::pair<ServoID, double>> goalAngles = calculateHeadJoints(lookVector);

        Eigen::Vector2d result;
        for (auto& angle : goalAngles) {
            if (angle.first == ServoID::HEAD_PITCH) {
                result.y() = angle.second;
            }
            else if (angle.first == ServoID::HEAD_YAW) {
                result.x() = angle.second;
            }
        }
        return result;
    }

    /*! Get search points which keep everything in view.
    Returns vector of Eigen::Vector2d
    */
    std::vector<Eigen::Vector2d> HeadBehaviourSoccer::getSearchPoints(const KinematicsModel& /*unused*/,
                                                                      const Balls& fixationObjects,
                                                                      const SearchType& sType,
                                                                      const Sensors& /*unused*/,
                                                                      const Image::Lens& lens) {
        // If there is nothing of interest, we search fot points of interest
        // log("getting search points");
        if (fixationObjects.balls.empty()) {
            // log("getting search points 2");
            // Lost searches are normalised in terms of the FOV
            std::vector<Eigen::Vector2d> scaledResults;
            // scaledResults.push_back(utility::motion::kinematics::headAnglesToSeeGroundPoint(kinematicsModel,
            // lastLocBall.position,sensors));
            for (auto& p : searches[sType]) {
                // log("adding search point", p.t());
                // old angles thing
                // Interpolate between max and min allowed angles with -1 = min and 1 = max
                // auto angles = Eigen::Vector2d(((max_yaw - min_yaw) * p[0] + max_yaw + min_yaw) / 2,
                //                                    ((max_pitch - min_pitch) * p[1] + max_pitch + min_pitch) /
                //                                    2);

                // New absolute referencing
                Eigen::Vector2d angles = p * M_PI / 180;

                scaledResults.push_back(angles);
            }
            return scaledResults;
        }

        Quad<double, 2, 1> boundingBox = getScreenAngularBoundingBox(fixationObjects);

        std::vector<Eigen::Vector2d> viewPoints;
        if (lens.fov == 0) {
            log<NUClear::WARN>("NO CAMERA PARAMETERS LOADED!!");
        }
        // Get points which keep everything on screen with padding
        float view_padding_radians = fractional_view_padding * lens.fov;
        // 1
        Eigen::Vector2d padding = {view_padding_radians, view_padding_radians};
        Eigen::Vector2d tr      = boundingBox.getBottomLeft() - padding + Eigen::Vector2d(lens.fov, lens.fov) * 0.5;
        // 2
        padding            = {view_padding_radians, -view_padding_radians};
        Eigen::Vector2d br = boundingBox.getTopLeft() - padding + Eigen::Vector2d(lens.fov, -lens.fov) * 0.5;
        // 3
        padding            = {-view_padding_radians, -view_padding_radians};
        Eigen::Vector2d bl = boundingBox.getTopRight() - padding - Eigen::Vector2d(lens.fov, lens.fov) * 0.5;
        // 4
        padding            = {-view_padding_radians, view_padding_radians};
        Eigen::Vector2d tl = boundingBox.getBottomRight() - padding + Eigen::Vector2d(-lens.fov, lens.fov) * 0.5;

        // Interpolate between max and min allowed angles with -1 = min and 1 = max
        std::vector<Eigen::Vector2d> searchPoints;
        for (auto& p : searches[SearchType::FIND_ADDITIONAL_OBJECTS]) {
            const float x = p[0];
            const float y = p[1];
            searchPoints.emplace_back(
                ((1 - x) * (1 - y) * bl + (1 - x) * (1 + y) * tl + (1 + x) * (1 + y) * tr + (1 + x) * (1 - y) * br)
                / 4);
        }

        return searchPoints;
    }

    std::vector<Eigen::Vector2d> HeadBehaviourSoccer::getSearchPoints(const KinematicsModel& /*unused*/,
                                                                      const Goals& fixationObjects,
                                                                      const SearchType& sType,
                                                                      const Sensors& /*unused*/,
                                                                      const Image::Lens& lens) {
        // If there is nothing of interest, we search fot points of interest
        // log("getting search points");
        if (fixationObjects.goals.empty()) {
            // log("getting search points 2");
            // Lost searches are normalised in terms of the FOV
            std::vector<Eigen::Vector2d> scaledResults;
            // scaledResults.push_back(utility::motion::kinematics::headAnglesToSeeGroundPoint(kinematicsModel,
            // lastLocBall.position,sensors));
            for (auto& p : searches[sType]) {
                // log("adding search point", p.t());
                // old angles thing
                // Interpolate between max and min allowed angles with -1 = min and 1 = max
                // auto angles = Eigen::Vector2d(((max_yaw - min_yaw) * p[0] + max_yaw + min_yaw) / 2,
                //                                    ((max_pitch - min_pitch) * p[1] + max_pitch + min_pitch) /
                //                                    2);

                // New absolute referencing
                Eigen::Vector2d angles = p * M_PI / 180;

                scaledResults.push_back(angles);
            }
            return scaledResults;
        }

        Quad<double, 2, 1> boundingBox = getScreenAngularBoundingBox(fixationObjects);

        std::vector<Eigen::Vector2d> viewPoints;
        if (lens.fov == 0) {
            log<NUClear::WARN>("NO CAMERA PARAMETERS LOADED!!");
        }
        // Get points which keep everything on screen with padding
        double view_padding_radians = fractional_view_padding * lens.fov;
        // 1
        Eigen::Vector2d padding = {view_padding_radians, view_padding_radians};
        Eigen::Vector2d tr      = boundingBox.getBottomLeft() - padding + Eigen::Vector2d(lens.fov, lens.fov) * 0.5;
        // 2
        padding            = {view_padding_radians, -view_padding_radians};
        Eigen::Vector2d br = boundingBox.getTopLeft() - padding + Eigen::Vector2d(lens.fov, -lens.fov) * 0.5;
        // 3
        padding            = {-view_padding_radians, -view_padding_radians};
        Eigen::Vector2d bl = boundingBox.getTopRight() - padding - Eigen::Vector2d(lens.fov, lens.fov) * 0.5;
        // 4
        padding            = {-view_padding_radians, view_padding_radians};
        Eigen::Vector2d tl = boundingBox.getBottomRight() - padding + Eigen::Vector2d(-lens.fov, lens.fov) * 0.5;

        // Interpolate between max and min allowed angles with -1 = min and 1 = max
        std::vector<Eigen::Vector2d> searchPoints;
        for (auto& p : searches[SearchType::FIND_ADDITIONAL_OBJECTS]) {
            const float x = p[0];
            const float y = p[1];
            searchPoints.emplace_back(
                ((1 - x) * (1 - y) * bl + (1 - x) * (1 + y) * tl + (1 + x) * (1 + y) * tr + (1 + x) * (1 - y) * br)
                / 4);
        }

        return searchPoints;
    }

    Ball HeadBehaviourSoccer::combineVisionObjects(const Balls& ob) {
        if (ob.balls.empty()) {
            log<NUClear::WARN>(
                "HeadBehaviourSoccer::combineVisionBalls - Attempted to combine zero vision objects into one.");
            return Ball();
        }
        Quad<double, 2, 1> q = getScreenAngularBoundingBox(ob);
        Ball v               = ob.balls.at(0);
        v.screen_angular     = q.getCentre().cast<float>();
        v.angular_size       = q.getSize().cast<float>();
        return v;
    }

    Quad<double, 2, 1> HeadBehaviourSoccer::getScreenAngularBoundingBox(const Balls& ob) {
        std::vector<Eigen::Vector2d> boundingPoints;
        for (const auto& ball : ob.balls) {
            boundingPoints.emplace_back((ball.screen_angular + ball.angular_size * 0.5).cast<double>());
            boundingPoints.emplace_back((ball.screen_angular - ball.angular_size * 0.5).cast<double>());
        }
        return Quad<double, 2, 1>::getBoundingBox(boundingPoints);
    }


    Goal HeadBehaviourSoccer::combineVisionObjects(const Goals& ob) {
        if (ob.goals.empty()) {
            log<NUClear::WARN>(
                "HeadBehaviourSoccer::combineVisionObjects - Attempted to combine zero vision objects into "
                "one.");
            return Goal();
        }
        return ob.goals.at(0);
    }

    Quad<double, 2, 1> HeadBehaviourSoccer::getScreenAngularBoundingBox(const Goals& ob) {
        std::vector<Eigen::Vector2d> boundingPoints;
        for (const auto& goal : ob.goals) {
            boundingPoints.emplace_back((goal.screen_angular + goal.angular_size * 0.5).cast<double>());
            boundingPoints.emplace_back((goal.screen_angular - goal.angular_size * 0.5).cast<double>());
        }
        return Quad<double, 2, 1>::getBoundingBox(boundingPoints);
    }

    bool HeadBehaviourSoccer::orientationHasChanged(const message::input::Sensors& sensors) {
        Eigen::Matrix3d diff = Eigen::Affine3d(sensors.Htw).inverse().rotation() * Rtw;
        Eigen::Quaterniond quat(diff);

        // Max and min prevent nand error, presumably due to computational limitations
        double angle = 2.0 * utility::math::angle::acos_clamped(std::min(1.0, std::max(quat.w(), -1.0)));

        return std::abs(angle) > replan_angle_threshold;
    }

}  // namespace module::behaviour::skills
