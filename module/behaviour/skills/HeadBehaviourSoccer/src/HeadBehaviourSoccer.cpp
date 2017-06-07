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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "HeadBehaviourSoccer.h"

#include <string>

#include "extension/Configuration.h"

#include "message/motion/GetupCommand.h"
#include "message/motion/HeadCommand.h"
#include "message/motion/KinematicsModels.h"

#include "utility/input/ServoID.h"
#include "utility/math/coordinates.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/vision.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/yaml_expression.h"

namespace module {
    namespace behaviour{
        namespace skills {

        using extension::Configuration;

        using utility::nubugger::graph;

        using message::vision::Goal;
        using message::vision::Ball;
        using message::vision::VisionObject;
        // using message::localisation::Ball;
        using message::localisation::Self;
        using LocBall = message::localisation::Ball;
        using message::input::Sensors;
        using message::motion::HeadCommand;

        using message::input::CameraParameters;
        using message::motion::ExecuteGetup;
        using message::motion::KillGetup;
        using message::motion::KinematicsModel;

        using utility::math::coordinates::sphericalToCartesian;
        using utility::motion::kinematics::calculateCameraLookJoints;
        using utility::math::matrix::Rotation3D;
        using utility::math::matrix::Transform3D;
        using utility::math::geometry::Quad;
        using utility::math::geometry::UnitQuaternion;
        using utility::math::vision::objectDirectionFromScreenAngular;
        using utility::math::vision::screenAngularFromObjectDirection;

        using ServoID = utility::input::ServoID;

        using message::behaviour::SoccerObjectPriority;
        using SearchType = message::behaviour::SoccerObjectPriority::SearchType;

            HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment))
                , max_yaw(0.0f)
                , min_yaw(0.0f)
                , max_pitch(0.0f)
                , min_pitch(0.0f)
                , replan_angle_threshold(0.0f)
                , lastPlanOrientation()
                , cam()
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

                on<Configuration>("HeadBehaviourSoccer.yaml").then("Head Behaviour Soccer Config", [this] (const Configuration& config) {
                    lastPlanUpdate = NUClear::clock::now();
                    timeLastObjectSeen = NUClear::clock::now();

                    //Config HeadBehaviourSoccer.yaml
                    fractional_view_padding = config["fractional_view_padding"].as<double>();

                    search_timeout_ms = config["search_timeout_ms"].as<float>();

                    fractional_angular_update_threshold = config["fractional_angular_update_threshold"].as<float>();

                    headSearcher.setSwitchTime(config["fixation_time_ms"].as<float>());

                    oscillate_search = config["oscillate_search"].as<bool>();

                    //Note that these are actually modified later and are hence camelcase
                    ballPriority = config["initial"]["priority"]["ball"].as<int>();
                    goalPriority = config["initial"]["priority"]["goal"].as<int>();

                    replan_angle_threshold = config["replan_angle_threshold"].as<float>();

                    pitch_plan_threshold = config["pitch_plan_threshold"].as<float>() * M_PI / 180.0f;
                    pitch_plan_value = config["pitch_plan_value"].as<float>() * M_PI / 180.0f;

                    //Load searches:
                    for(auto& search : config["searches"].config){
                        SearchType s(search["search_type"].as<std::string>());
                        searches[s] = std::vector<Eigen::Vector2d>();
                        for (auto& p : search["points"]){
                            searches[s].push_back(p.as<Eigen::Vector2d>());
                        }
                    }


                });


                // TODO: remove this horrible code
                // Check to see if we are currently in the process of getting up.
                on<Trigger<ExecuteGetup>>().then([this] {
                    isGettingUp = true;
                });

                // Check to see if we have finished getting up.
                on<Trigger<KillGetup>>().then([this] {
                    isGettingUp = false;
                });


                on<Trigger<SoccerObjectPriority>, Sync<HeadBehaviourSoccer>>().then("Head Behaviour Soccer - Set priorities", [this] (const SoccerObjectPriority& p) {
                    ballPriority = p.ball;
                    goalPriority = p.goal;
                    searchType = p.searchType;
                });

                auto initialPriority = std::make_unique<SoccerObjectPriority>();
                initialPriority->ball = 1;
                emit(initialPriority);


                on<Trigger<Sensors>,
                    Optional<With<std::vector<Ball>>>,
                    Optional<With<std::vector<Goal>>>,
                    Optional<With<LocBall>>,
                    With<KinematicsModel>,
                    With<CameraParameters>,
                    Single,
                    Sync<HeadBehaviourSoccer>
                  >().then("Head Behaviour Main Loop", [this] ( const Sensors& sensors,
                                                        std::shared_ptr<const std::vector<Ball>> vballs,
                                                        std::shared_ptr<const std::vector<Goal>> vgoals,
                                                        std::shared_ptr<const LocBall> locBall,
                                                        const KinematicsModel& kinematicsModel,
                                                        const CameraParameters& cam_
                                                        ) {

                    max_yaw = kinematicsModel.head.MAX_YAW;
                    min_yaw = kinematicsModel.head.MIN_YAW;
                    max_pitch = kinematicsModel.head.MAX_PITCH;
                    min_pitch = kinematicsModel.head.MIN_PITCH;

                    // std::cout << "Seen: Balls: " <<
                    // ((vballs != nullptr) ? std::to_string(int(vballs->size())) : std::string("null")) <<
                    // "Goals: " <<
                    // ((vgoals != nullptr) ? std::to_string(int(vgoals->size())) : std::string("null")) << std::endl;

                    //TODO: pass camera parameters around instead of this hack storage
                    cam = cam_;

                    if(locBall) {
                        locBallReceived = true;
                        lastLocBall = *locBall;
                    }
                    auto now = NUClear::clock::now();

                    bool objectsMissing = false;

                    //Get the list of objects which are currently visible
                    std::vector<Ball> ballFixationObjects = getFixationObjects(vballs, objectsMissing);
                    std::vector<Goal> goalFixationObjects = getFixationObjects(vgoals, objectsMissing);

                    //Determine state transition variables
                    bool lost = ((ballFixationObjects.size() <= 0) && (goalFixationObjects.size() <= 0));
                    //Do we need to update our plan?
                    bool updatePlan = !isGettingUp && ((lastBallPriority != ballPriority) || (lastGoalPriority != goalPriority));
                    //Has it been a long time since we have seen anything of interest?
                    bool searchTimedOut = std::chrono::duration_cast<std::chrono::milliseconds>(now - timeLastObjectSeen).count() > search_timeout_ms;
                    //Did the object move in IMUspace?
                    bool objectMoved = false;

                    bool ballMaxPriority = (ballPriority == std::max(ballPriority, goalPriority));

                    // log("updatePlan", updatePlan);
                    // log("lost", lost);
                    // log("isGettingUp", isGettingUp);
                    // log("searchType", int(searchType));
                    // log("headSearcher.size()", headSearcher.size());

                    //State execution

                    //Get robot heat to body transform
                    Rotation3D orientation, headToBodyRotation;
                    if(!lost){
                        //We need to transform our view points to orientation space
                        if (ballMaxPriority)
                        {
                            headToBodyRotation = Transform3D(convert<double, 4, 4>(ballFixationObjects[0].visObject.sensors->forwardKinematics.at(ServoID::HEAD_PITCH))).rotation();
                            orientation        = Transform3D(convert<double, 4, 4>(ballFixationObjects[0].visObject.sensors->world)).rotation().i();
                        }
                        else
                        {
                            headToBodyRotation = Transform3D(convert<double, 4, 4>(goalFixationObjects[0].visObject.sensors->forwardKinematics.at(ServoID::HEAD_PITCH))).rotation();
                            orientation        = Transform3D(convert<double, 4, 4>(goalFixationObjects[0].visObject.sensors->world)).rotation().i();
                        }
                    } else {
                        headToBodyRotation = Transform3D(convert<double, 4, 4>(sensors.forwardKinematics.at(ServoID::HEAD_PITCH))).rotation();
                        orientation        = Transform3D(convert<double, 4, 4>(sensors.world)).rotation().i();
                    }
                    Rotation3D headToIMUSpace = orientation * headToBodyRotation;

                    //If objects visible, check current centroid to see if it moved
                    if(!lost){
                        Eigen::Vector2d currentCentroid = Eigen::Vector2d(0,0);
                        if (ballMaxPriority)
                        {
                            for(auto& ob : ballFixationObjects){
                                currentCentroid += ob.visObject.screenAngular / float(ballFixationObjects.size());
                            }
                        }
                        else
                        {
                            for(auto& ob : goalFixationObjects){
                                currentCentroid += ob.visObject.screenAngular / float(goalFixationObjects.size());
                            }
                        }
                        Eigen::Vector2d currentCentroid_world = getIMUSpaceDirection(kinematicsModel,currentCentroid,headToIMUSpace);
                        //If our objects have moved, we need to replan
                        if(arma::norm(currentCentroid_world - lastCentroid) >= fractional_angular_update_threshold * std::fmax(cam.FOV[0],cam.FOV[1]) / 2.0){
                            objectMoved = true;
                            lastCentroid = currentCentroid_world;
                        }
                    }

                    //State Transitions
                    if(!isGettingUp){
                        switch(state){
                            case FIXATION:
                                if(lost) {
                                    state = WAIT;
                                } else if(objectMoved) {
                                    updatePlan = true;
                                }
                                break;
                            case WAIT:
                                if(!lost){
                                    state = FIXATION;
                                    updatePlan = true;
                                }
                                else if(searchTimedOut){
                                    state = SEARCH;
                                    updatePlan = true;
                                }
                                break;
                            case SEARCH:
                                if(!lost){
                                    state = FIXATION;
                                    updatePlan = true;
                                }
                                break;
                        }
                    }

                    //If we arent getting up, then we can update the plan if necessary
                    if(updatePlan){
                        if(lost){
                            lastPlanOrientation = Transform3D(convert<double, 4, 4>(sensors.world)).rotation();
                        }
                        if (ballMaxPriority)
                        {
                            updateHeadPlan(kinematicsModel, ballFixationObjects, objectsMissing, sensors, headToIMUSpace);
                        }

                        else
                        {
                            updateHeadPlan(kinematicsModel, goalFixationObjects, objectsMissing, sensors, headToIMUSpace);
                        }
                    }

                    //Update searcher
                    headSearcher.update(oscillate_search);
                    //Emit new result if possible
                    if(headSearcher.newGoal()){
                        //Emit result
                        Eigen::Vector2d direction = headSearcher.getState();
                        std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                        command->yaw = direction[0];
                        command->pitch = direction[1];
                        command->robotSpace = (state == SEARCH);
                        // log("head angles robot space :", command->robotSpace);
                        emit(std::move(command));
                    }

                    lastGoalPriority = goalPriority;
                    lastBallPriority = ballPriority;

                });


            }

            std::vector<Ball> HeadBehaviourSoccer::getFixationObjects(std::shared_ptr<const std::vector<Ball>> vballs, bool& search){

                auto now = NUClear::clock::now();
                std::vector<Ball> fixationObjects;

                int maxPriority = std::max(std::max(ballPriority, goalPriority),0);
                if(ballPriority == goalPriority) log<NUClear::WARN>("HeadBehaviourSoccer - Multiple object searching currently not supported properly.");

                //TODO: make this a loop over a list of objects or something
                //Get balls
                if(ballPriority == maxPriority){
                    if(vballs && vballs->size() > 0){
                        //Fixate on ball
                        timeLastObjectSeen = now;
                        auto& ball = vballs->at(0);
                        fixationObjects.push_back(ball);
                    } else {
                        search = true;
                    }
                }

                return fixationObjects;
            }

            std::vector<Goal> HeadBehaviourSoccer::getFixationObjects(std::shared_ptr<const std::vector<Goal>> vgoals, bool& search){

                auto now = NUClear::clock::now();
                std::vector<Goal> fixationObjects;

                int maxPriority = std::max(std::max(ballPriority, goalPriority),0);
                if(ballPriority == goalPriority) log<NUClear::WARN>("HeadBehaviourSoccer - Multiple object searching currently not supported properly.");

                //TODO: make this a loop over a list of objects or something
                //Get goals
                if(goalPriority == maxPriority){
                    if(vgoals && vgoals->size() > 0){
                        //Fixate on goals and lines and other landmarks
                        timeLastObjectSeen = now;
                        std::set<Goal::Side> visiblePosts;
                        //TODO treat goals as one object
                        std::vector<Goal> goals;
                        for (auto& goal : *vgoals){
                            visiblePosts.insert(goal.side);
                            goals.push_back(goal);
                        }
                        fixationObjects.push_back(combineVisionObjects(goals));
                        search = (visiblePosts.find(Goal::Side::LEFT) == visiblePosts.end() ||//If left post not visible or
                                  visiblePosts.find(Goal::Side::RIGHT) == visiblePosts.end());//right post not visible, then we need to search for the other goal post
                    } else {
                        search = true;
                    }
                }

                return fixationObjects;
            }


            void HeadBehaviourSoccer::updateHeadPlan(const KinematicsModel& kinematicsModel, const std::vector<Ball>& fixationObjects, const bool& search, const Sensors& sensors, const Rotation3D& headToIMUSpace){
                std::vector<Eigen::Vector2d> fixationPoints;
                std::vector<Eigen::Vector2d> fixationSizes;
                Eigen::VectorXd centroid = {0,0};
                Eigen::Vector2d currentPos = {sensors.servo.at(ServoID::HEAD_YAW).presentPosition, sensors.servo.at(ServoID::HEAD_PITCH).presentPosition};

                for(uint i = 0; i < fixationObjects.size(); i++){
                    //TODO: fix arma meat errors here
                    //Should be vec2 (yaw,pitch)
                    fixationPoints.push_back(Eigen::Vector2d(fixationObjects[i].visObject.screenAngular[0],fixationObjects[i].visObject.screenAngular[1]));
                    fixationSizes.push_back(Eigen::Vector2d(fixationObjects[i].visObject.angularSize[0],fixationObjects[i].visObject.angularSize[1]));
                    //Average here as it is more elegant than an if statement checking if size==0 at the end
                    centroid += arma::vec(fixationObjects[i].visObject.screenAngular) / (fixationObjects.size());
                }

                //If there are objects to find
                if(search){
                    fixationPoints = getSearchPoints(kinematicsModel,fixationObjects, searchType, sensors);
                }

                if(fixationPoints.size() <= 0){
                    log("FOUND NO POINTS TO LOOK AT! - ARE THE SEARCHES PROPERLY CONFIGURED IN HEADBEHAVIOURSOCCER.YAML?");
                }

                //Transform to IMU space including compensation for current head pose
                if(!search){
                    for(auto& p : fixationPoints){
                        p = getIMUSpaceDirection(kinematicsModel,p, headToIMUSpace);
                    }
                    currentPos = getIMUSpaceDirection(kinematicsModel,currentPos, headToIMUSpace);
                }

                headSearcher.replaceSearchPoints(fixationPoints, currentPos);
            }

            void HeadBehaviourSoccer::updateHeadPlan(const KinematicsModel& kinematicsModel, const std::vector<Goal>& fixationObjects, const bool& search, const Sensors& sensors, const Rotation3D& headToIMUSpace){
                std::vector<Eigen::Vector2d> fixationPoints;
                std::vector<Eigen::Vector2d> fixationSizes;
                Eigen::VectorXd centroid = {0,0};
                Eigen::Vector2d currentPos;
                for (const auto& servo : sensors.servo)
                {
                    if (servo.id == ServoID::HEAD_YAW)
                    {
                        currentPos[0] = servo.presentPosition;
                    }
                    if (servo.id == ServoID::HEAD_PITCH)
                    {
                        currentPos[1] = servo.presentPosition;
                    }
                }

                for(uint i = 0; i < fixationObjects.size(); i++){
                    //TODO: fix arma meat errors here
                    //Should be vec2 (yaw,pitch)
                    fixationPoints.push_back(Eigen::Vector2d(fixationObjects[i].visObject.screenAngular[0],fixationObjects[i].visObject.screenAngular[1]));
                    fixationSizes.push_back(Eigen::Vector2d(fixationObjects[i].visObject.angularSize[0],fixationObjects[i].visObject.angularSize[1]));
                    //Average here as it is more elegant than an if statement checking if size==0 at the end
                    centroid += arma::vec(fixationObjects[i].visObject.screenAngular) / (fixationObjects.size());
                }

                //If there are objects to find
                if(search){
                    fixationPoints = getSearchPoints(kinematicsModel,fixationObjects, searchType, sensors);
                }

                if(fixationPoints.size() <= 0){
                    log("FOUND NO POINTS TO LOOK AT! - ARE THE SEARCHES PROPERLY CONFIGURED IN HEADBEHAVIOURSOCCER.YAML?");
                }

                //Transform to IMU space including compensation for current head pose
                if(!search){
                    for(auto& p : fixationPoints){
                        p = getIMUSpaceDirection(kinematicsModel,p, headToIMUSpace);
                    }
                    currentPos = getIMUSpaceDirection(kinematicsModel,currentPos, headToIMUSpace);
                }

                headSearcher.replaceSearchPoints(fixationPoints, currentPos);
            }

            Eigen::Vector2d HeadBehaviourSoccer::getIMUSpaceDirection(const KinematicsModel& kinematicsModel, const Eigen::Vector2d& screenAngles, Rotation3D headToIMUSpace){

                // Eigen::Vector3d lookVectorFromHead = objectDirectionFromScreenAngular(screenAngles);
                Eigen::Vector3d lookVectorFromHead = sphericalToCartesian({1,screenAngles[0],screenAngles[1]});//This is an approximation relying on the robots small FOV
                //Remove pitch from matrix if we are adjusting search points

                //Rotate target angles to World space
                Eigen::Vector3d lookVector = headToIMUSpace * lookVectorFromHead;
                //Compute inverse kinematics for head direction angles
                std::vector< std::pair<ServoID, float> > goalAngles = calculateCameraLookJoints(kinematicsModel, lookVector);

                Eigen::Vector2d result;
                for(auto& angle : goalAngles){
                    if(angle.first == ServoID::HEAD_PITCH){
                        result[1] = angle.second;
                    } else if(angle.first == ServoID::HEAD_YAW){
                        result[0] = angle.second;
                    }
                }
                return result;
            }

            /*! Get search points which keep everything in view.
            Returns vector of Eigen::Vector2d
            */
            std::vector<Eigen::Vector2d> HeadBehaviourSoccer::getSearchPoints(const KinematicsModel&,std::vector<Ball> fixationObjects, SearchType sType, const Sensors&){
                    //If there is nothing of interest, we search fot points of interest
                    // log("getting search points");
                    if(fixationObjects.size() == 0){
                        // log("getting search points 2");
                        //Lost searches are normalised in terms of the FOV
                        std::vector<Eigen::Vector2d> scaledResults;
                        //scaledResults.push_back(utility::motion::kinematics::headAnglesToSeeGroundPoint(kinematicsModel, lastLocBall.position,sensors));
                        for(auto& p : searches[sType]){
                            // log("adding search point", p.t());
                            //old angles thing
                            //Interpolate between max and min allowed angles with -1 = min and 1 = max
                            //auto angles = Eigen::Vector2d(((max_yaw - min_yaw) * p[0] + max_yaw + min_yaw) / 2,
                            //                                    ((max_pitch - min_pitch) * p[1] + max_pitch + min_pitch) / 2);

                            //New absolute referencing
                            Eigen::Vector2d angles = p * M_PI / 180;
                            // if(std::fabs(sensors.world.rotation().pitch()) < pitch_plan_threshold){
                                // Eigen::Vector3d lookVectorFromHead = sphericalToCartesian({1,angles[0],angles[1]});//This is an approximation relying on the robots small FOV


                                //TODO: Fix trying to look underneath and behind self!!


                                // Eigen::Vector3d adjustedLookVector = lookVectorFromHead;
                                //TODO: fix:
                                // Eigen::Vector3d adjustedLookVector = Rotation3D::createRotationX(sensors.world.rotation().pitch()) * lookVectorFromHead;
                                // Eigen::Vector3d adjustedLookVector = Rotation3D::createRotationY(-pitch_plan_value) * lookVectorFromHead;
                                // std::vector< std::pair<ServoID, float> > goalAngles = calculateCameraLookJoints(kinematicsModel, adjustedLookVector);

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

                    std::vector<Eigen::Vector2d> viewPoints;
                    if(cam.FOV.norm() == 0){
                        log<NUClear::WARN>("NO CAMERA PARAMETERS LOADED!!");
                    }
                    //Get points which keep everything on screen with padding
                    float view_padding_radians = fractional_view_padding * std::fmax(cam.FOV[0],cam.FOV[1]);
                    //1
                    Eigen::Vector2d padding = {view_padding_radians,view_padding_radians};
                    Eigen::Vector2d tr = boundingBox.getBottomLeft() - padding + cam.FOV / 2.0;
                    //2
                    padding = {view_padding_radians,-view_padding_radians};
                    Eigen::Vector2d br = boundingBox.getTopLeft() - padding + Eigen::Vector2d(cam.FOV[0],-cam.FOV[1]) / 2.0;
                    //3
                    padding = {-view_padding_radians,-view_padding_radians};
                    Eigen::Vector2d bl = boundingBox.getTopRight() - padding - cam.FOV / 2.0;
                    //4
                    padding = {-view_padding_radians,view_padding_radians};
                    Eigen::Vector2d tl = boundingBox.getBottomRight() - padding + Eigen::Vector2d(-cam.FOV[0],cam.FOV[1]) / 2.0;

                    //Interpolate between max and min allowed angles with -1 = min and 1 = max
                    std::vector<Eigen::Vector2d> searchPoints;
                    for(auto& p : searches[SearchType::FIND_ADDITIONAL_OBJECTS]){
                        float x = p[0];
                        float y = p[1];
                        searchPoints.push_back(( (1-x)*(1-y)*bl+
                                                 (1-x)*(1+y)*tl+
                                                 (1+x)*(1+y)*tr+
                                                 (1+x)*(1-y)*br )/4);
                    }

                    return searchPoints;

            }

            std::vector<Eigen::Vector2d> HeadBehaviourSoccer::getSearchPoints(const KinematicsModel&,std::vector<Goal> fixationObjects, SearchType sType, const Sensors&){
                    //If there is nothing of interest, we search fot points of interest
                    // log("getting search points");
                    if(fixationObjects.size() == 0){
                        // log("getting search points 2");
                        //Lost searches are normalised in terms of the FOV
                        std::vector<Eigen::Vector2d> scaledResults;
                        //scaledResults.push_back(utility::motion::kinematics::headAnglesToSeeGroundPoint(kinematicsModel, lastLocBall.position,sensors));
                        for(auto& p : searches[sType]){
                            // log("adding search point", p.t());
                            //old angles thing
                            //Interpolate between max and min allowed angles with -1 = min and 1 = max
                            //auto angles = Eigen::Vector2d(((max_yaw - min_yaw) * p[0] + max_yaw + min_yaw) / 2,
                            //                                    ((max_pitch - min_pitch) * p[1] + max_pitch + min_pitch) / 2);

                            //New absolute referencing
                            Eigen::Vector2d angles = p * M_PI / 180;
                            // if(std::fabs(sensors.world.rotation().pitch()) < pitch_plan_threshold){
                                // Eigen::Vector3d lookVectorFromHead = sphericalToCartesian({1,angles[0],angles[1]});//This is an approximation relying on the robots small FOV


                                //TODO: Fix trying to look underneath and behind self!!


                                // Eigen::Vector3d adjustedLookVector = lookVectorFromHead;
                                //TODO: fix:
                                // Eigen::Vector3d adjustedLookVector = Rotation3D::createRotationX(sensors.world.rotation().pitch()) * lookVectorFromHead;
                                // Eigen::Vector3d adjustedLookVector = Rotation3D::createRotationY(-pitch_plan_value) * lookVectorFromHead;
                                // std::vector< std::pair<ServoID, float> > goalAngles = calculateCameraLookJoints(kinematicsModel, adjustedLookVector);

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

                    std::vector<Eigen::Vector2d> viewPoints;
                    if(cam.FOV.norm() == 0){
                        log<NUClear::WARN>("NO CAMERA PARAMETERS LOADED!!");
                    }
                    //Get points which keep everything on screen with padding
                    float view_padding_radians = fractional_view_padding * std::fmax(cam.FOV[0],cam.FOV[1]);
                    //1
                    Eigen::Vector2d padding = {view_padding_radians,view_padding_radians};
                    Eigen::Vector2d tr = boundingBox.getBottomLeft() - padding + cam.FOV / 2.0;
                    //2
                    padding = {view_padding_radians,-view_padding_radians};
                    Eigen::Vector2d br = boundingBox.getTopLeft() - padding + Eigen::Vector2d(cam.FOV[0],-cam.FOV[1]) / 2.0;
                    //3
                    padding = {-view_padding_radians,-view_padding_radians};
                    Eigen::Vector2d bl = boundingBox.getTopRight() - padding - cam.FOV / 2.0;
                    //4
                    padding = {-view_padding_radians,view_padding_radians};
                    Eigen::Vector2d tl = boundingBox.getBottomRight() - padding + Eigen::Vector2d(-cam.FOV[0],cam.FOV[1]) / 2.0;

                    //Interpolate between max and min allowed angles with -1 = min and 1 = max
                    std::vector<Eigen::Vector2d> searchPoints;
                    for(auto& p : searches[SearchType::FIND_ADDITIONAL_OBJECTS]){
                        float x = p[0];
                        float y = p[1];
                        searchPoints.push_back(( (1-x)*(1-y)*bl+
                                                 (1-x)*(1+y)*tl+
                                                 (1+x)*(1+y)*tr+
                                                 (1+x)*(1-y)*br )/4);
                    }

                    return searchPoints;

            }

            Ball HeadBehaviourSoccer::combineVisionObjects(const std::vector<Ball>& ob){
                if(ob.size() == 0){
                    log<NUClear::WARN>("HeadBehaviourSoccer::combineVisionBalls - Attempted to combine zero vision objects into one.");
                    return VisionObject();
                }
                Quad q = getScreenAngularBoundingBox(ob);
                Ball v = ob[0];
                v.visObject.screenAngular = q.getCentre();
                v.visObject.angularSize = q.getSize();
                return v;
            }

            Quad HeadBehaviourSoccer::getScreenAngularBoundingBox(const std::vector<Ball>& ob){
                std::vector<Eigen::Vector2d> boundingPoints;
                for(uint i = 0; i< ob.size(); i++){
                    boundingPoints.push_back(ob[i].visObject.screenAngular+ob[i].visObject.angularSize / 2);
                    boundingPoints.push_back(ob[i].visObject.screenAngular-ob[i].visObject.angularSize / 2);
                }
                return Quad::getBoundingBox(boundingPoints);
            }


            Goal HeadBehaviourSoccer::combineVisionObjects(const std::vector<Goal>& ob){
                if(ob.size() == 0){
                    log<NUClear::WARN>("HeadBehaviourSoccer::combineVisionObjects - Attempted to combine zero vision objects into one.");
                    return VisionObject();
                }
                Quad q = getScreenAngularBoundingBox(ob);
                Goal v = ob[0];
                v.visObject.screenAngular = q.getCentre();
                v.visObject.angularSize = q.getSize();
                return v;
            }

            Quad HeadBehaviourSoccer::getScreenAngularBoundingBox(const std::vector<Goal>& ob){
                std::vector<Eigen::Vector2d> boundingPoints;
                for(uint i = 0; i< ob.size(); i++){
                    boundingPoints.push_back(ob[i].visObject.screenAngular+ob[i].visObject.angularSize / 2);
                    boundingPoints.push_back(ob[i].visObject.screenAngular-ob[i].visObject.angularSize / 2);
                }
                return Quad::getBoundingBox(boundingPoints);
            }


            bool HeadBehaviourSoccer::orientationHasChanged(const message::input::Sensors& sensors){
                Rotation3D diff = Transform3D(convert<double, 4, 4>(sensors.world)).rotation().i() * lastPlanOrientation;
                UnitQuaternion quat = UnitQuaternion(diff);
                float angle = quat.getAngle();
                return std::fabs(angle) > replan_angle_threshold;
            }

        }  // motion
    } //behaviour
}  // modules
