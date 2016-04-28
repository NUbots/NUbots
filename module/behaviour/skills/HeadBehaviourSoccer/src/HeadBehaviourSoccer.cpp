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
#include "message/support/Configuration.h"
#include "message/motion/HeadCommand.h"
#include "utility/math/coordinates.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/geometry/Quad.h"
#include "utility/math/vision.h"
#include "message/motion/GetupCommand.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"

#include "utility/nubugger/NUhelpers.h"

#include <string>


namespace module {
    namespace behaviour{
        namespace skills {

        using utility::nubugger::graph;

        using message::vision::Goal;
        using message::vision::Ball;
        using message::vision::VisionObject;
        using message::support::Configuration;
        // using message::localisation::Ball;
        using message::localisation::Self;
        using LocBall = message::localisation::Ball;
        using message::input::Sensors;
        using message::motion::HeadCommand;

        using message::input::CameraParameters;
        using message::motion::ExecuteGetup;
        using message::motion::KillGetup;

        using utility::math::coordinates::sphericalToCartesian;
        using utility::motion::kinematics::calculateHeadJoints;
        using utility::motion::kinematics::DarwinModel;
        using utility::math::matrix::Rotation3D;
        using utility::math::geometry::Quad;
        using utility::math::geometry::UnitQuaternion;
        using utility::math::vision::objectDirectionFromScreenAngular;
        using utility::math::vision::screenAngularFromObjectDirection;

        using message::input::ServoID;

        using message::behaviour::SoccerObjectPriority;
        using message::behaviour::SearchType;
        using message::behaviour::searchTypeFromString;

            HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment) :
            Reactor(std::move(environment)),
            lastCentroid({0,0}),
            lostAndSearching(false),
            lostLastTime(false),
            lastBallPriority(0),
            lastGoalPriority(0) {

                on<Configuration>("HeadBehaviourSoccer.yaml").then("Head Behaviour Soccer Config", [this] (const Configuration& config) {
                    lastPlanUpdate = NUClear::clock::now();
                    timeLastObjectSeen = NUClear::clock::now();
                    max_yaw = utility::motion::kinematics::DarwinModel::Head::MAX_YAW;
                    min_yaw = utility::motion::kinematics::DarwinModel::Head::MIN_YAW;
                    max_pitch = utility::motion::kinematics::DarwinModel::Head::MAX_PITCH;
                    min_pitch = utility::motion::kinematics::DarwinModel::Head::MIN_PITCH;

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

                    //Load searches:
                    for(auto& search : config["searches"]){
                        SearchType s = searchTypeFromString(search["search_type"].as<std::string>());
                        searches[s] = std::vector<arma::vec2>();
                        for (auto& p : search["points"]){
                            searches[s].push_back(p.as<arma::vec2>());
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


                on<Trigger<CameraParameters>>().then("Head Behaviour - Load CameraParameters",[this] (const CameraParameters& cam_) {
                    cam = cam_;
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
                    Single,
                    Sync<HeadBehaviourSoccer>
                  >().then("Head Behaviour Main Loop", [this] ( const Sensors& sensors,
                                                        std::shared_ptr<const std::vector<Ball>> vballs,
                                                        std::shared_ptr<const std::vector<Goal>> vgoals,
                                                        std::shared_ptr<const LocBall> locBall
                                                        ) {

                    // std::cout << "Seen: Balls: " <<
                    // ((vballs != nullptr) ? std::to_string(int(vballs->size())) : std::string("null")) << 
                    // "Goals: " <<
                    // ((vgoals != nullptr) ? std::to_string(int(vgoals->size())) : std::string("null")) << std::endl;
                   
                    if(locBall) {
                        locBallReceived = true;
                        lastLocBall = *locBall;
                    }

                    bool search = false;

                    int maxPriority = std::max(std::max(ballPriority,goalPriority),0);

                    if(ballPriority == goalPriority) log<NUClear::WARN>("HeadBehaviourSoccer - Multiple object searching currently not supported properly.");

                    std::vector<VisionObject> fixationObjects;

                    auto now = NUClear::clock::now();

                    //TODO: make this a loop over a list of objects or something
                    if(ballPriority == maxPriority){
                        if(vballs != NULL && vballs->size() > 0){
                            //Fixate on ball
                            timeLastObjectSeen = now;
                            auto& ball = vballs->at(0);
                            fixationObjects.push_back(VisionObject(ball));
                        } else {
                            search = true;
                        }
                    }
                    if(goalPriority == maxPriority){
                        if(vgoals != NULL && vgoals->size() > 0){
                            //Fixate on goals and lines and other landmarks
                            timeLastObjectSeen = now;
                            std::set<Goal::Side> visiblePosts;
                            //TODO treat goals as one object
                            std::vector<VisionObject> goals;
                            for (auto& goal : *vgoals){
                                visiblePosts.insert(goal.side);
                                goals.push_back(VisionObject(goal));
                            }
                            fixationObjects.push_back(combineVisionObjects(goals));
                            search = (visiblePosts.find(Goal::Side::LEFT) == visiblePosts.end() ||//If left post not visible or
                                      visiblePosts.find(Goal::Side::RIGHT) == visiblePosts.end());//right post not visible, then we need to search for the other goal post
                        } else {
                            search = true;
                        }
                    }
                    bool lost = fixationObjects.size() <= 0;
                    bool found = !lost && lostLastTime;
                    //Do we need to update our plan?
                    bool updatePlan = headSearcher.size() <= 1 || found || (lastBallPriority != ballPriority) || (lastGoalPriority != goalPriority) ; //bool(Priorities have changed)

                    // log("updatePlan", updatePlan);
                    // log("lost", lost);
                    // log("isGettingUp", isGettingUp);
                    // log("searchType", int(searchType));
                    // log("headSearcher.size()", headSearcher.size());
                    
                    //Get robot pose
                    Rotation3D orientation, headToBodyRotation;
                    if(!lost){
                        //We need to transform our view points to orientation space
                        lostAndSearching = false;
                        headToBodyRotation = fixationObjects[0].sensors->forwardKinematics.at(ServoID::HEAD_PITCH).rotation();
                        orientation = fixationObjects[0].sensors->orientation.i();
                    } else {
                        headToBodyRotation = sensors.forwardKinematics.at(ServoID::HEAD_PITCH).rotation();
                        orientation = sensors.orientation.i();
                    }
                    Rotation3D headToIMUSpace = orientation * headToBodyRotation;

                    //Check current centroid
                    if(!lost){
                        arma::vec2 currentCentroid = arma::vec2({0,0});
                        for(auto& ob : fixationObjects){
                            currentCentroid += ob.screenAngular / float(fixationObjects.size());
                        }
                        arma::vec2 currentCentroid_world = getIMUSpaceDirection(currentCentroid,headToIMUSpace);
                        //If our objects have moved, we need to replan
                        if(arma::norm(currentCentroid_world - lastCentroid) >= fractional_angular_update_threshold * std::fmax(cam.FOV[0],cam.FOV[1]) / 2.0){
                            updatePlan = true;
                            lastCentroid = currentCentroid_world;
                            //std::cout << "Replanning due to object movement." << std::endl;
                        }
                    }

                    //If we lost what we are searching for.
                    if(!lostAndSearching && std::chrono::duration_cast<std::chrono::milliseconds>(now - timeLastObjectSeen).count() > search_timeout_ms ){
                        lostAndSearching = true;
                        updatePlan = true;
                        lastCentroid = {99999,99999};//reset centroid to impossible value to trigger reset TODO: find a better way
                    }
                    //TODO!! - Put search pattern back in IMU space and make replanning work
                    else if(false && lostAndSearching && orientationHasChanged(sensors)){
                        // log("orientation has changed: replanning");
                        updatePlan = true;
                        lastCentroid = {99999,99999};//reset centroid to impossible value to trigger reset TODO: find a better way
                    }

                    if(updatePlan && !isGettingUp){
                        if(lost){
                            lastPlanOrientation = sensors.orientation;
                        }
                        updateHeadPlan(fixationObjects, search, sensors, headToIMUSpace);
                    }

                    //Update state machine
                    headSearcher.update(oscillate_search);
                    //Emit new result if possible
                    if(headSearcher.newGoal()){
                        //Emit result
                        arma::vec2 direction = headSearcher.getState();
                        std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                        command->yaw = direction[0];
                        command->pitch = direction[1];
                        command->robotSpace = search;
                        emit(std::move(command));
                    }

                    lastGoalPriority = goalPriority;
                    lastBallPriority = ballPriority;

                    lostLastTime = lost;
                });


            }

            void HeadBehaviourSoccer::updateHeadPlan(const std::vector<VisionObject>& fixationObjects, const bool& search, const Sensors& sensors, const Rotation3D& headToIMUSpace){
                std::vector<arma::vec2> fixationPoints;
                std::vector<arma::vec2> fixationSizes;
                arma::vec centroid = {0,0};
                for(uint i = 0; i < fixationObjects.size(); i++){
                    //TODO: fix arma meat errors here
                    //Should be vec2 (yaw,pitch)
                    fixationPoints.push_back(arma::vec({fixationObjects[i].screenAngular[0],fixationObjects[i].screenAngular[1]}));
                    fixationSizes.push_back(arma::vec({fixationObjects[i].angularSize[0],fixationObjects[i].angularSize[1]}));
                    //Average here as it is more elegant than an if statement checking if size==0 at the end
                    centroid += arma::vec(fixationObjects[i].screenAngular) / (fixationObjects.size());
                }

                if(search){
                    fixationPoints = getSearchPoints(fixationObjects, searchType, sensors);
                }

                if(fixationPoints.size() <= 0){
                    log("FOUND NO POINTS TO LOOK AT! - ARE THE SEARCHES PROPERLY CONFIGURED IN HEADBEHAVIOURSOCCER.YAML?");
                }
                auto currentPos = arma::vec2({sensors.servos.at(int(ServoID::HEAD_YAW)).presentPosition,sensors.servos.at(int(ServoID::HEAD_PITCH)).presentPosition});
                if(!search){
                    for(auto& p : fixationPoints){
                        p = getIMUSpaceDirection(p, headToIMUSpace);
                    }
                    currentPos = getIMUSpaceDirection(currentPos, headToIMUSpace);
                }

                headSearcher.replaceSearchPoints(fixationPoints, currentPos);
            }

            arma::vec2 HeadBehaviourSoccer::getIMUSpaceDirection(const arma::vec2& screenAngles, Rotation3D headToIMUSpace){

                // arma::vec3 lookVectorFromHead = objectDirectionFromScreenAngular(screenAngles);
                arma::vec3 lookVectorFromHead = sphericalToCartesian({1,screenAngles[0],screenAngles[1]});//This is an approximation relying on the robots small FOV
                //Remove pitch from matrix if we are adjusting search points

                //Rotate target angles to World space
                arma::vec3 lookVector = headToIMUSpace * lookVectorFromHead;
                //Compute inverse kinematics for head direction angles
                std::vector< std::pair<ServoID, float> > goalAngles = calculateHeadJoints<DarwinModel>(lookVector);

                arma::vec2 result;
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
            Returns vector of arma::vec2
            */
            std::vector<arma::vec2> HeadBehaviourSoccer::getSearchPoints(std::vector<VisionObject> fixationObjects, SearchType sType, const Sensors& sensors){
                    //If there is nothing of interest, we search fot points of interest
                    // log("getting search points");
                    if(fixationObjects.size() == 0){
                        // log("getting search points 2");
                        //Lost searches are normalised in terms of the FOV
                        std::vector<arma::vec2> scaledResults;
                        //scaledResults.push_back(utility::motion::kinematics::headAnglesToSeeGroundPoint<DarwinModel>(lastLocBall.position,sensors));
                        for(auto& p : searches[sType]){
                            // log("adding search point", p.t());
                            //old angles thing
                            //Interpolate between max and min allowed angles with -1 = min and 1 = max
                            //auto angles = arma::vec2({((max_yaw - min_yaw) * p[0] + max_yaw + min_yaw) / 2,
                            //                                    ((max_pitch - min_pitch) * p[1] + max_pitch + min_pitch) / 2});

                            //New absolute referencing 
                            arma::vec2 angles = p * M_PI / 180;
                            if(std::fabs(sensors.orientation.pitch()) < pitch_plan_threshold){
                                arma::vec3 lookVectorFromHead = sphericalToCartesian({1,angles[0],angles[1]});//This is an approximation relying on the robots small FOV


                                //TODO: Fix trying to look underneath and behind self!!


                                arma::vec3 adjustedLookVector = Rotation3D::createRotationY(sensors.orientation.pitch()) * lookVectorFromHead;
                                std::vector< std::pair<ServoID, float> > goalAngles = calculateHeadJoints<DarwinModel>(adjustedLookVector);

                                for(auto& angle : goalAngles){
                                    if(angle.first == ServoID::HEAD_PITCH){
                                        angles[1] = angle.second;
                                    } else if(angle.first == ServoID::HEAD_YAW){
                                        angles[0] = angle.second;
                                    }
                                }
                            }
                            // emit(graph("IMUSpace Head Lost Angles", angles));

                            scaledResults.push_back(angles);
                        }
                        return scaledResults;
                    }

                    Quad boundingBox = getScreenAngularBoundingBox(fixationObjects);

                    std::vector<arma::vec2> viewPoints;
                    if(arma::norm(cam.FOV) == 0){
                        log<NUClear::WARN>("NO CAMERA PARAMETERS LOADED!!");
                    }
                    //Get points which keep everything on screen with padding
                    float view_padding_radians = fractional_view_padding * std::fmax(cam.FOV[0],cam.FOV[1]);
                    //1
                    arma::vec2 padding = {view_padding_radians,view_padding_radians};
                    arma::vec2 tr = boundingBox.getBottomLeft() - padding + cam.FOV / 2.0;
                    //2
                    padding = {view_padding_radians,-view_padding_radians};
                    arma::vec2 br = boundingBox.getTopLeft() - padding + arma::vec({cam.FOV[0],-cam.FOV[1]}) / 2.0;
                    //3
                    padding = {-view_padding_radians,-view_padding_radians};
                    arma::vec2 bl = boundingBox.getTopRight() - padding - cam.FOV / 2.0;
                    //4
                    padding = {-view_padding_radians,view_padding_radians};
                    arma::vec2 tl = boundingBox.getBottomRight() - padding + arma::vec({-cam.FOV[0],cam.FOV[1]}) / 2.0;

                    //Interpolate between max and min allowed angles with -1 = min and 1 = max
                    std::vector<arma::vec2> searchPoints;
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

            VisionObject HeadBehaviourSoccer::combineVisionObjects(const std::vector<VisionObject>& ob){
                if(ob.size() == 0){
                    log<NUClear::WARN>("HeadBehaviourSoccer::combineVisionObjects - Attempted to combine zero vision objects into one.");
                    return VisionObject();
                }
                Quad q = getScreenAngularBoundingBox(ob);
                VisionObject v = ob[0];
                v.screenAngular = q.getCentre();
                v.angularSize = q.getSize();
                return v;
            }

            Quad HeadBehaviourSoccer::getScreenAngularBoundingBox(const std::vector<VisionObject>& ob){
                std::vector<arma::vec2> boundingPoints;
                for(uint i = 0; i< ob.size(); i++){
                    boundingPoints.push_back(ob[i].screenAngular+ob[i].angularSize / 2);
                    boundingPoints.push_back(ob[i].screenAngular-ob[i].angularSize / 2);
                }
                return Quad::getBoundingBox(boundingPoints);
            }


            bool HeadBehaviourSoccer::orientationHasChanged(const message::input::Sensors& sensors){
                Rotation3D diff = sensors.orientation.i() * lastPlanOrientation;
                UnitQuaternion quat = UnitQuaternion(diff);
                float angle = quat.getAngle();
                return std::fabs(angle) > replan_angle_threshold;
            }


        }  // motion
    } //behaviour
}  // modules
