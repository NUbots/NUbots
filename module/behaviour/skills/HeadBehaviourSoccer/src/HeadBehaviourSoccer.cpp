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

#include "HeadBehaviourSoccer.h"

#include <string>

#include "extension/Configuration.h"
#include "message/localisation/Field.h"
#include "message/motion/GetupCommand.h"
#include "message/motion/HeadCommand.h"
#include "message/vision/Ball.h"
#include "message/vision/Goal.h"
#include "utility/input/ServoID.h"
#include "utility/math/coordinates.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"


namespace module {
namespace behaviour {
    namespace skills {

        using extension::Configuration;

        using utility::nusight::graph;

        using message::vision::Ball;
        using message::vision::Balls;
        using message::vision::Goal;
        using message::vision::Goals;
        // using message::localisation::Ball;
        using message::localisation::Field;
        using LocBall = message::localisation::Ball;
        using message::input::Image;
        using message::input::Sensors;
        using message::motion::HeadCommand;

        using message::motion::ExecuteGetup;
        using message::motion::KillGetup;
        using message::motion::KinematicsModel;

        using utility::math::coordinates::sphericalToCartesian;
        using utility::math::geometry::Quad;
        using utility::math::geometry::UnitQuaternion;
        using utility::math::matrix::Rotation3D;
        using utility::math::matrix::Transform3D;
        using utility::motion::kinematics::calculateCameraLookJoints;

        using ServoID = utility::input::ServoID;

        using message::behaviour::SoccerObjectPriority;
        using SearchType = message::behaviour::SoccerObjectPriority::SearchType;

        inline arma::vec screenAngularFromObjectDirection(const arma::vec& v) {
            return {std::atan2(v[1], v[0]), std::atan2(v[2], v[0])};
        }

        inline arma::vec objectDirectionFromScreenAngular(const arma::vec& screen_angular) {
            if (std::fmod(std::fabs(screen_angular[0]), M_PI) == M_PI_2
                || std::fmod(std::fabs(screen_angular[1]), M_PI) == M_PI_2) {
                return {0, 0, 0};
            }
            double tanTheta        = std::tan(screen_angular[0]);
            double tanPhi          = std::tan(screen_angular[1]);
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
            : Reactor(std::move(environment))
            , max_yaw(0.0f)
            , min_yaw(0.0f)
            , max_pitch(0.0f)
            , min_pitch(0.0f)
            , replan_angle_threshold(0.0f)
            , lastPlanOrientation()
            , pitch_plan_threshold(0.0f)
            , fractional_view_padding(0.0)
            , search_timeout_ms(0.0f)
            , fractional_angular_update_threshold(0.0f)
            , oscillate_search(false)
            , lastLocBall()
            , searches()
            , headSearcher()
            , lastPlanUpdate()
            , timeLastObjectSeen()
            , lastCentroid(arma::fill::zeros) {

            on<Configuration>("HeadBehaviourSoccer.yaml")
                .then("Head Behaviour Soccer Config", [this](const Configuration& config) {
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
                    for (auto& search : config["searches"].config) {
                        SearchType s(search["search_type"].as<std::string>());
                        searches[s] = std::vector<arma::vec2>();
                        for (auto& p : search["points"]) {
                            searches[s].push_back(p.as<arma::vec2>());
                        }
                    }
                });


            // TODO: remove this horrible code
            // Check to see if we are currently in the process of getting up.
            on<Trigger<ExecuteGetup>>().then([this] { isGettingUp = true; });

            // Check to see if we have finished getting up.
            on<Trigger<KillGetup>>().then([this] { isGettingUp = false; });


            on<Trigger<SoccerObjectPriority>, Sync<HeadBehaviourSoccer>>().then(
                "Head Behaviour Soccer - Set priorities", [this](const SoccerObjectPriority& p) {
                    ballPriority = p.ball;
                    goalPriority = p.goal;
                    searchType   = p.searchType;
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
                             std::shared_ptr<const Balls> vballs,
                             std::shared_ptr<const Goals> vgoals,
                             std::shared_ptr<const LocBall> locBall,
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
                          auto now = NUClear::clock::now();

                          bool objectsMissing = false;

                          // Get the list of objects which are currently visible
                          Balls ballFixationObjects = getFixationObjects(vballs, objectsMissing);
                          Goals goalFixationObjects = getFixationObjects(vgoals, objectsMissing);

                          // Determine state transition variables
                          bool lost =
                              ((ballFixationObjects.balls.size() <= 0) && (goalFixationObjects.goals.size() <= 0));
                          // Do we need to update our plan?
                          bool updatePlan =
                              !isGettingUp
                              && ((lastBallPriority != ballPriority) || (lastGoalPriority != goalPriority));
                          // Has it been a long time since we have seen anything of interest?
                          bool searchTimedOut =
                              std::chrono::duration_cast<std::chrono::milliseconds>(now - timeLastObjectSeen).count()
                              > search_timeout_ms;
                          // Did the object move in IMUspace?
                          bool objectMoved = false;

                          bool ballMaxPriority = (ballPriority == std::max(ballPriority, goalPriority));

                          // log("updatePlan", updatePlan);
                          // log("lost", lost);
                          // log("isGettingUp", isGettingUp);
                          // log("searchType", int(searchType));
                          // log("headSearcher.size()", headSearcher.size());

                          // State execution

                          // Get robot heat to body transform
                          Rotation3D orientation, headToBodyRotation;
                          if (!lost) {
                              // We need to transform our view points to orientation space
                              if (ballMaxPriority) {
                                  Eigen::Matrix4d Htc = sensors.Htw * ballFixationObjects.Hcw.inverse();
                                  headToBodyRotation  = Transform3D(convert(Htc)).rotation();
                                  orientation         = Transform3D(convert(ballFixationObjects.Hcw)).rotation().i();
                              }
                              else {
                                  Eigen::Matrix4d Htc = sensors.Htw * goalFixationObjects.Hcw.inverse();
                                  headToBodyRotation  = Transform3D(convert(Htc)).rotation();
                                  orientation         = Transform3D(convert(goalFixationObjects.Hcw)).rotation().i();
                              }
                          }
                          else {
                              Eigen::Matrix4d Htc = sensors.forward_kinematics[ServoID::HEAD_PITCH];
                              headToBodyRotation  = Transform3D(convert(Htc)).rotation();
                              orientation         = Transform3D(convert(sensors.Htw)).rotation().i();
                          }
                          Rotation3D headToIMUSpace = orientation * headToBodyRotation;

                          // If objects visible, check current centroid to see if it moved
                          if (!lost) {
                              arma::vec2 currentCentroid = arma::vec2({0, 0});
                              if (ballMaxPriority) {
                                  for (auto& ob : ballFixationObjects.balls) {
                                      currentCentroid += arma::conv_to<arma::vec>::from(convert(ob.screen_angular))
                                                         / float(ballFixationObjects.balls.size());
                                  }
                              }
                              else {
                                  for (auto& ob : goalFixationObjects.goals) {
                                      currentCentroid += arma::conv_to<arma::vec>::from(convert(ob.screen_angular))
                                                         / float(goalFixationObjects.goals.size());
                                  }
                              }
                              arma::vec2 currentCentroid_world =
                                  getIMUSpaceDirection(kinematicsModel, currentCentroid, headToIMUSpace);
                              // If our objects have moved, we need to replan
                              if (arma::norm(currentCentroid_world - lastCentroid)
                                  >= fractional_angular_update_threshold
                                         * std::fmax(image.lens.fov[0], image.lens.fov[1]) / 2.0) {
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
                                  lastPlanOrientation = Transform3D(convert(sensors.Htw)).rotation();
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
                              arma::vec2 direction                 = headSearcher.getState();
                              std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                              command->yaw                         = direction[0];
                              command->pitch                       = direction[1];
                              command->robotSpace                  = (state == SEARCH);
                              // log("head angles robot space :", command->robotSpace);
                              emit(std::move(command));
                          }

                          lastGoalPriority = goalPriority;
                          lastBallPriority = ballPriority;
                      });
        }

        Balls HeadBehaviourSoccer::getFixationObjects(std::shared_ptr<const Balls> vballs, bool& search) {

            auto now = NUClear::clock::now();
            Balls fixationObjects;

            int maxPriority = std::max(std::max(ballPriority, goalPriority), 0);
            if (ballPriority == goalPriority) {
                log<NUClear::WARN>("HeadBehaviourSoccer - Multiple object searching currently not supported properly.");
            }

            // TODO: make this a loop over a list of objects or something
            // Get balls
            if (ballPriority == maxPriority) {
                if (vballs && vballs->balls.size() > 0) {
                    // Fixate on ball
                    timeLastObjectSeen = now;
                    auto& ball         = vballs->balls.at(0);
                    fixationObjects.balls.push_back(ball);
                }
                else {
                    search = true;
                }
            }

            return fixationObjects;
        }

        Goals HeadBehaviourSoccer::getFixationObjects(std::shared_ptr<const Goals> vgoals, bool& search) {

            auto now = NUClear::clock::now();
            Goals fixationObjects;

            int maxPriority = std::max(std::max(ballPriority, goalPriority), 0);
            if (ballPriority == goalPriority) {
                log<NUClear::WARN>("HeadBehaviourSoccer - Multiple object searching currently not supported properly.");
            }

            // TODO: make this a loop over a list of objects or something
            // Get goals
            if (goalPriority == maxPriority) {
                if (vgoals && vgoals->goals.size() > 0) {
                    // Fixate on goals and lines and other landmarks
                    timeLastObjectSeen = now;
                    std::set<Goal::Side> visiblePosts;
                    // TODO treat goals as one object
                    Goals goals;
                    for (auto& goal : vgoals->goals) {
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
                                                 const Rotation3D& headToIMUSpace,
                                                 const Image::Lens& lens) {
            std::vector<arma::vec2> fixationPoints;
            std::vector<arma::vec2> fixationSizes;
            arma::vec2 currentPos = {sensors.servo.at(ServoID::HEAD_YAW).present_position,
                                     sensors.servo.at(ServoID::HEAD_PITCH).present_position};

            for (uint i = 0; i < fixationObjects.balls.size(); i++) {
                // TODO: fix arma meat errors here
                // Should be vec2 (yaw,pitch)
                fixationPoints.push_back(arma::vec(
                    {fixationObjects.balls.at(i).screen_angular[0], fixationObjects.balls.at(i).screen_angular[1]}));
                fixationSizes.push_back(arma::vec(
                    {fixationObjects.balls.at(i).angular_size[0], fixationObjects.balls.at(i).angular_size[1]}));
            }

            // If there are objects to find
            if (search) {
                fixationPoints = getSearchPoints(kinematicsModel, fixationObjects, searchType, sensors, lens);
            }

            if (fixationPoints.size() <= 0) {
                log("FOUND NO POINTS TO LOOK AT! - ARE THE SEARCHES PROPERLY CONFIGURED IN HEADBEHAVIOURSOCCER.YAML?");
            }

            // Transform to IMU space including compensation for current head pose
            if (!search) {
                for (auto& p : fixationPoints) {
                    p = getIMUSpaceDirection(kinematicsModel, p, headToIMUSpace);
                }
                currentPos = getIMUSpaceDirection(kinematicsModel, currentPos, headToIMUSpace);
            }

            headSearcher.replaceSearchPoints(fixationPoints, currentPos);
        }

        void HeadBehaviourSoccer::updateHeadPlan(const KinematicsModel& kinematicsModel,
                                                 const Goals& fixationObjects,
                                                 const bool& search,
                                                 const Sensors& sensors,
                                                 const Rotation3D& headToIMUSpace,
                                                 const Image::Lens& lens) {
            std::vector<arma::vec2> fixationPoints;
            std::vector<arma::vec2> fixationSizes;
            arma::vec2 currentPos;
            for (const auto& servo : sensors.servo) {
                if (servo.id == ServoID::HEAD_YAW) {
                    currentPos[0] = servo.present_position;
                }
                if (servo.id == ServoID::HEAD_PITCH) {
                    currentPos[1] = servo.present_position;
                }
            }

            for (uint i = 0; i < fixationObjects.goals.size(); i++) {
                // TODO: fix arma meat errors here
                // Should be vec2 (yaw,pitch)
                fixationPoints.push_back(arma::vec(
                    {fixationObjects.goals.at(i).screen_angular[0], fixationObjects.goals.at(i).screen_angular[1]}));
                fixationSizes.push_back(arma::vec(
                    {fixationObjects.goals.at(i).angular_size[0], fixationObjects.goals.at(i).angular_size[1]}));
            }

            // If there are objects to find
            if (search) {
                fixationPoints = getSearchPoints(kinematicsModel, fixationObjects, searchType, sensors, lens);
            }

            if (fixationPoints.size() <= 0) {
                log("FOUND NO POINTS TO LOOK AT! - ARE THE SEARCHES PROPERLY CONFIGURED IN HEADBEHAVIOURSOCCER.YAML?");
            }

            // Transform to IMU space including compensation for current head pose
            if (!search) {
                for (auto& p : fixationPoints) {
                    p = getIMUSpaceDirection(kinematicsModel, p, headToIMUSpace);
                }
                currentPos = getIMUSpaceDirection(kinematicsModel, currentPos, headToIMUSpace);
            }

            headSearcher.replaceSearchPoints(fixationPoints, currentPos);
        }

        arma::vec2 HeadBehaviourSoccer::getIMUSpaceDirection(const KinematicsModel& kinematicsModel,
                                                             const arma::vec2& screenAngles,
                                                             Rotation3D headToIMUSpace) {

            // arma::vec3 lookVectorFromHead = objectDirectionFromScreenAngular(screenAngles);
            arma::vec3 lookVectorFromHead = sphericalToCartesian(
                {1, screenAngles[0], screenAngles[1]});  // This is an approximation relying on the robots small FOV
            // Remove pitch from matrix if we are adjusting search points

            // Rotate target angles to World space
            arma::vec3 lookVector = headToIMUSpace * lookVectorFromHead;
            // Compute inverse kinematics for head direction angles
            std::vector<std::pair<ServoID, float>> goalAngles = calculateCameraLookJoints(kinematicsModel, lookVector);

            arma::vec2 result;
            for (auto& angle : goalAngles) {
                if (angle.first == ServoID::HEAD_PITCH) {
                    result[1] = angle.second;
                }
                else if (angle.first == ServoID::HEAD_YAW) {
                    result[0] = angle.second;
                }
            }
            return result;
        }

        /*! Get search points which keep everything in view.
        Returns vector of arma::vec2
        */
        std::vector<arma::vec2> HeadBehaviourSoccer::getSearchPoints(const KinematicsModel&,
                                                                     Balls fixationObjects,
                                                                     SearchType sType,
                                                                     const Sensors&,
                                                                     const Image::Lens& lens) {
            // If there is nothing of interest, we search fot points of interest
            // log("getting search points");
            if (fixationObjects.balls.size() == 0) {
                // log("getting search points 2");
                // Lost searches are normalised in terms of the FOV
                std::vector<arma::vec2> scaledResults;
                // scaledResults.push_back(utility::motion::kinematics::headAnglesToSeeGroundPoint(kinematicsModel,
                // lastLocBall.position,sensors));
                for (auto& p : searches[sType]) {
                    // log("adding search point", p.t());
                    // old angles thing
                    // Interpolate between max and min allowed angles with -1 = min and 1 = max
                    // auto angles = arma::vec2({((max_yaw - min_yaw) * p[0] + max_yaw + min_yaw) / 2,
                    //                                    ((max_pitch - min_pitch) * p[1] + max_pitch + min_pitch) /
                    //                                    2});

                    // New absolute referencing
                    arma::vec2 angles = p * M_PI / 180;
                    // if(std::fabs(sensors.Htw.rotation().pitch()) < pitch_plan_threshold){
                    // arma::vec3 lookVectorFromHead = sphericalToCartesian({1,angles[0],angles[1]});//This is an
                    // approximation relying on the robots small FOV


                    // TODO: Fix trying to look underneath and behind self!!


                    // arma::vec3 adjustedLookVector = lookVectorFromHead;
                    // TODO: fix:
                    // arma::vec3 adjustedLookVector = Rotation3D::createRotationX(sensors.Htw.rotation().pitch()) *
                    // lookVectorFromHead;
                    // arma::vec3 adjustedLookVector = Rotation3D::createRotationY(-pitch_plan_value) *
                    // lookVectorFromHead;
                    // std::vector< std::pair<ServoID, float> > goalAngles = calculateCameraLookJoints(kinematicsModel,
                    // adjustedLookVector);

                    // for(auto& angle : goalAngles){
                    //     if(angle.first == ServoID::HEAD_PITCH){
                    //         angles[1] = angle.second;
                    //     } else if(angle.first == ServoID::HEAD_YAW){
                    //         angles[0] = angle.second;
                    //     }
                    // }
                    // log("goalAngles",angles.t());
                    // }
                    // emit(graph("IMUSpace Head Lost Angles", angles));

                    scaledResults.push_back(angles);
                }
                return scaledResults;
            }

            Quad boundingBox = getScreenAngularBoundingBox(fixationObjects);

            std::vector<arma::vec2> viewPoints;
            if (lens.fov.norm() == 0) {
                log<NUClear::WARN>("NO CAMERA PARAMETERS LOADED!!");
            }
            // Get points which keep everything on screen with padding
            float view_padding_radians = fractional_view_padding * std::fmax(lens.fov[0], lens.fov[1]);
            // 1
            arma::vec2 padding = {view_padding_radians, view_padding_radians};
            arma::vec2 tr      = boundingBox.getBottomLeft() - padding + convert(lens.fov) / 2.0;
            // 2
            padding       = {view_padding_radians, -view_padding_radians};
            arma::vec2 br = boundingBox.getTopLeft() - padding + arma::vec({lens.fov[0], -lens.fov[1]}) / 2.0;
            // 3
            padding       = {-view_padding_radians, -view_padding_radians};
            arma::vec2 bl = boundingBox.getTopRight() - padding - convert(lens.fov) / 2.0;
            // 4
            padding       = {-view_padding_radians, view_padding_radians};
            arma::vec2 tl = boundingBox.getBottomRight() - padding + arma::vec({-lens.fov[0], lens.fov[1]}) / 2.0;

            // Interpolate between max and min allowed angles with -1 = min and 1 = max
            std::vector<arma::vec2> searchPoints;
            for (auto& p : searches[SearchType::FIND_ADDITIONAL_OBJECTS]) {
                float x = p[0];
                float y = p[1];
                searchPoints.push_back(
                    ((1 - x) * (1 - y) * bl + (1 - x) * (1 + y) * tl + (1 + x) * (1 + y) * tr + (1 + x) * (1 - y) * br)
                    / 4);
            }

            return searchPoints;
        }

        std::vector<arma::vec2> HeadBehaviourSoccer::getSearchPoints(const KinematicsModel&,
                                                                     Goals fixationObjects,
                                                                     SearchType sType,
                                                                     const Sensors&,
                                                                     const Image::Lens& lens) {
            // If there is nothing of interest, we search fot points of interest
            // log("getting search points");
            if (fixationObjects.goals.size() == 0) {
                // log("getting search points 2");
                // Lost searches are normalised in terms of the FOV
                std::vector<arma::vec2> scaledResults;
                // scaledResults.push_back(utility::motion::kinematics::headAnglesToSeeGroundPoint(kinematicsModel,
                // lastLocBall.position,sensors));
                for (auto& p : searches[sType]) {
                    // log("adding search point", p.t());
                    // old angles thing
                    // Interpolate between max and min allowed angles with -1 = min and 1 = max
                    // auto angles = arma::vec2({((max_yaw - min_yaw) * p[0] + max_yaw + min_yaw) / 2,
                    //                                    ((max_pitch - min_pitch) * p[1] + max_pitch + min_pitch) /
                    //                                    2});

                    // New absolute referencing
                    arma::vec2 angles = p * M_PI / 180;
                    // if(std::fabs(sensors.Htw.rotation().pitch()) < pitch_plan_threshold){
                    // arma::vec3 lookVectorFromHead = sphericalToCartesian({1,angles[0],angles[1]});//This is an
                    // approximation relying on the robots small FOV


                    // TODO: Fix trying to look underneath and behind self!!


                    // arma::vec3 adjustedLookVector = lookVectorFromHead;
                    // TODO: fix:
                    // arma::vec3 adjustedLookVector = Rotation3D::createRotationX(sensors.Htw.rotation().pitch()) *
                    // lookVectorFromHead;
                    // arma::vec3 adjustedLookVector = Rotation3D::createRotationY(-pitch_plan_value) *
                    // lookVectorFromHead;
                    // std::vector< std::pair<ServoID, float> > goalAngles = calculateCameraLookJoints(kinematicsModel,
                    // adjustedLookVector);

                    // for(auto& angle : goalAngles){
                    //     if(angle.first == ServoID::HEAD_PITCH){
                    //         angles[1] = angle.second;
                    //     } else if(angle.first == ServoID::HEAD_YAW){
                    //         angles[0] = angle.second;
                    //     }
                    // }
                    // log("goalAngles",angles.t());
                    // }
                    // emit(graph("IMUSpace Head Lost Angles", angles));

                    scaledResults.push_back(angles);
                }
                return scaledResults;
            }

            Quad boundingBox = getScreenAngularBoundingBox(fixationObjects);

            std::vector<arma::vec2> viewPoints;
            if (lens.fov.norm() == 0) {
                log<NUClear::WARN>("NO CAMERA PARAMETERS LOADED!!");
            }
            // Get points which keep everything on screen with padding
            float view_padding_radians = fractional_view_padding * std::fmax(lens.fov[0], lens.fov[1]);
            // 1
            arma::vec2 padding = {view_padding_radians, view_padding_radians};
            arma::vec2 tr      = boundingBox.getBottomLeft() - padding + convert(lens.fov) / 2.0;
            // 2
            padding       = {view_padding_radians, -view_padding_radians};
            arma::vec2 br = boundingBox.getTopLeft() - padding + arma::vec({lens.fov[0], -lens.fov[1]}) / 2.0;
            // 3
            padding       = {-view_padding_radians, -view_padding_radians};
            arma::vec2 bl = boundingBox.getTopRight() - padding - convert(lens.fov) / 2.0;
            // 4
            padding       = {-view_padding_radians, view_padding_radians};
            arma::vec2 tl = boundingBox.getBottomRight() - padding + arma::vec({-lens.fov[0], lens.fov[1]}) / 2.0;

            // Interpolate between max and min allowed angles with -1 = min and 1 = max
            std::vector<arma::vec2> searchPoints;
            for (auto& p : searches[SearchType::FIND_ADDITIONAL_OBJECTS]) {
                float x = p[0];
                float y = p[1];
                searchPoints.push_back(
                    ((1 - x) * (1 - y) * bl + (1 - x) * (1 + y) * tl + (1 + x) * (1 + y) * tr + (1 + x) * (1 - y) * br)
                    / 4);
            }

            return searchPoints;
        }

        Ball HeadBehaviourSoccer::combineVisionObjects(const Balls& ob) {
            if (ob.balls.size() == 0) {
                log<NUClear::WARN>(
                    "HeadBehaviourSoccer::combineVisionBalls - Attempted to combine zero vision objects into one.");
                return Ball();
            }
            Quad q           = getScreenAngularBoundingBox(ob);
            Ball v           = ob.balls.at(0);
            v.screen_angular = convert(q.getCentre()).cast<float>();
            v.angular_size   = convert(q.getSize()).cast<float>();
            return v;
        }

        Quad HeadBehaviourSoccer::getScreenAngularBoundingBox(const Balls& ob) {
            std::vector<arma::vec2> boundingPoints;
            for (uint i = 0; i < ob.balls.size(); i++) {
                boundingPoints.push_back(arma::conv_to<arma::vec>::from(
                    convert(Eigen::Vector2f(ob.balls.at(i).screen_angular + ob.balls.at(i).angular_size / 2))));
                boundingPoints.push_back(arma::conv_to<arma::vec>::from(
                    convert(Eigen::Vector2f(ob.balls.at(i).screen_angular - ob.balls.at(i).angular_size / 2))));
            }
            return Quad::getBoundingBox(boundingPoints);
        }


        Goal HeadBehaviourSoccer::combineVisionObjects(const Goals& ob) {
            if (ob.goals.size() == 0) {
                log<NUClear::WARN>(
                    "HeadBehaviourSoccer::combineVisionObjects - Attempted to combine zero vision objects into one.");
                return Goal();
            }
            Quad q = getScreenAngularBoundingBox(ob);
            Goal v = ob.goals.at(0);
            return v;
        }

        Quad HeadBehaviourSoccer::getScreenAngularBoundingBox(const Goals& ob) {
            std::vector<arma::vec2> boundingPoints;
            for (uint i = 0; i < ob.goals.size(); i++) {
                boundingPoints.push_back(convert(Eigen::Vector2d(ob.goals.at(i).screen_angular.cast<double>()
                                                                 + ob.goals.at(i).angular_size.cast<double>() / 2)));
                boundingPoints.push_back(convert(Eigen::Vector2d(ob.goals.at(i).screen_angular.cast<double>()
                                                                 - ob.goals.at(i).angular_size.cast<double>() / 2)));
            }
            return Quad::getBoundingBox(boundingPoints);
        }


        bool HeadBehaviourSoccer::orientationHasChanged(const message::input::Sensors& sensors) {
            Rotation3D diff     = Transform3D(convert(sensors.Htw)).rotation().i() * lastPlanOrientation;
            UnitQuaternion quat = UnitQuaternion(diff);
            float angle         = quat.getAngle();
            return std::fabs(angle) > replan_angle_threshold;
        }

    }  // namespace skills
}  // namespace behaviour
}  // namespace module
