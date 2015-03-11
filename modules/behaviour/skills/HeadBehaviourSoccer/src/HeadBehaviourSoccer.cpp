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
#include "messages/localisation/FieldObject.h"
#include "messages/support/Configuration.h"
#include "messages/motion/HeadCommand.h"
#include "utility/math/coordinates.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/geometry/Quad.h"
#include "utility/math/vision.h"
#include "utility/support/yaml_armadillo.h"

#include "utility/nubugger/NUhelpers.h"


namespace modules {
    namespace behaviour{
        namespace skills {

        using utility::nubugger::graph;
        
        using messages::vision::Goal;
        using messages::vision::Ball;
        using messages::vision::VisionObject;
        using messages::support::Configuration;
        // using messages::localisation::Ball;
        using messages::localisation::Self;
        using messages::input::Sensors;
        using messages::motion::HeadCommand;
        using messages::input::CameraParameters;

        using utility::math::coordinates::sphericalToCartesian;
        using utility::motion::kinematics::calculateHeadJoints;
        using utility::motion::kinematics::DarwinModel;
        using utility::math::matrix::Rotation3D;
        using utility::math::geometry::Quad;
        using utility::math::vision::objectDirectionFromScreenAngular;
        using utility::math::vision::screenAngularFromObjectDirection;

        using messages::input::ServoID;

            HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment) : 
            Reactor(std::move(environment)),
            lastCentroid({0,0}),
            lostAndSearching(false),
            lostLastTime(false)
            {
                //do a little configurating
                on<Trigger<Configuration<HeadBehaviourSoccer>>>("Head Behaviour Soccer Config",[this] (const Configuration<HeadBehaviourSoccer>& config)
                {
                    lastPlanUpdate = NUClear::clock::now();
                    timeLastObjectSeen = NUClear::clock::now();
                    //Gains                    
                    view_padding_radians = config["view_padding_radians"].as<double>();      

                    tracking_p_gain = config["tracking_p_gain"].as<float>();

                    //Load searches:
                    for(auto& search : config["lost_searches"]){
                        SearchType s = searchTypeFromString(search["search_type"].as<std::string>());
                        lost_searches[s] = std::vector<arma::vec2>();
                        for (auto& p : search["points"]){
                            lost_searches[s].push_back(p.as<arma::vec2>());
                        }
                    }

                    max_yaw = utility::motion::kinematics::DarwinModel::Head::MAX_YAW;
                    min_yaw = utility::motion::kinematics::DarwinModel::Head::MIN_YAW;
                    max_pitch = utility::motion::kinematics::DarwinModel::Head::MAX_PITCH;
                    min_pitch = utility::motion::kinematics::DarwinModel::Head::MIN_PITCH;

                    headSearcher.setSwitchTime(config["fixation_time_ms"].as<float>());

                    plan_update_period = config["plan_update_period_ms"].as<float>();

                    angular_update_threshold = config["fractional_angular_update_threshold"].as<float>();

                    //TODO remove these configs
                    ballPriority = config["ballPriority"].as<int>();

                    goalPriority = config["goalPriority"].as<int>();


                });

                on<Trigger<CameraParameters>>("Head Behaviour - Load CameraParameters",[this] (const CameraParameters& cam_){
                    cam = cam_;
                });

                //TODO: trigger on balls with goals and check number of balls.
                on<
                    Trigger<std::vector<Ball>>,
                    With<Sensors>,
                    With<std::vector<Goal>>,
                    Options<Single>
                  >("Head Behaviour Main Loop",[this] ( const std::vector<Ball> vballs,
                                                        const Sensors& sensors,
                                                        const std::vector<Goal> vgoals
                                                        ) {

                    bool search = false;

                    int maxPriority = std::max(std::max(ballPriority,goalPriority),0);

                    std::vector<VisionObject> fixationObjects;

                    auto now = NUClear::clock::now();
                    //TODO: make this a loop over a list of objects or something
                    if(ballPriority == maxPriority){
                        if(vballs.size() > 0){
                            //Fixate on ball
                            timeLastObjectSeen = now;
                            auto& ball = vballs[0];
                            fixationObjects.push_back(VisionObject(ball));
                        } else {
                            search = true;
                        }
                    } 
                    if(goalPriority == maxPriority){
                        if(vgoals.size() > 0){
                            //Fixate on goals and lines and other landmarks
                            timeLastObjectSeen = now;
                            std::set<Goal::Side> visiblePosts;
                            //TODO treat goals as one object
                            std::vector<VisionObject> goals;
                            for (auto& goal : vgoals){
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
                    //Do we need to update our plan?
                    bool lost = fixationObjects.size() <= 0;
                    bool found = !lost && lostLastTime;
                    bool updatePlan = found; //bool(Priorities have changed)


                    //Get robot pose
                    Rotation3D orientation, headToBodyRotation;
                    if(!lost){
                        //We need to transform our view points to orientation space
                        lostAndSearching = false;
                        headToBodyRotation = fixationObjects[0].sensors->forwardKinematics.at(ServoID::HEAD_PITCH).rotation();
                        orientation = fixationObjects[0].sensors->orientation.i();
                    } else {
                        headToBodyRotation = arma::eye(3,3);
                        orientation = sensors.orientation.i();
                    }
                    Rotation3D headToIMUSpace = orientation * headToBodyRotation;                    

                    //Check current centroid
                    if(!lost){
                        arma::vec2 currentCentroid = arma::vec2({0,0});
                        for(auto& ob : fixationObjects){
                            currentCentroid = ob.screenAngular / float(fixationObjects.size());
                        }
                        arma::vec2 currentCentroid_world = getIMUSpaceDirection(currentCentroid,headToIMUSpace);
                        //If our objects have moved, we need to replan
                        if(arma::norm(currentCentroid_world - lastCentroid) >= angular_update_threshold * std::fmax(cam.FOV[0],cam.FOV[1]) / 2.0){                           
                            updatePlan = true;
                            lastCentroid = currentCentroid_world;
                        }
                    }

                    //If we lost what we are searching for.
                    if(!lostAndSearching && std::chrono::duration_cast<std::chrono::milliseconds>(now - timeLastObjectSeen).count() > plan_update_period ){
                        lostAndSearching = true;
                        updatePlan = true;
                        lastCentroid = {99999,99999};//reset centroid to impossible value to trigger reset TODO: find a better way
                    }

                    if(updatePlan){
                        updateHeadPlan(fixationObjects, search, sensors, headToIMUSpace);
                    }

                    //Update state machine
                    headSearcher.update();
                    //Emit new result if possible
                    if(headSearcher.newGoal()){
                        //Emit result
                        arma::vec2 direction = headSearcher.getState();
                        std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                        command->yaw = direction[0];
                        command->pitch = direction[1];
                        emit(std::move(command));
                    }

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
                    fixationPoints = getSearchPoints(fixationObjects, SearchType::LOW_FIRST);
                }

                if(fixationPoints.size() <= 0){
                    log("FOUND NO POINTS TO LOOK AT! - ARE THE SEARCHES PROPERLY CONFIGURED IN HEADBEHAVIOURSOCCER.YAML?");
                }

                for(auto& p : fixationPoints){
                    p = getIMUSpaceDirection(p,headToIMUSpace);
                }
                
                auto currentPos = arma::vec2({sensors.servos.at(int(ServoID::HEAD_YAW)).presentPosition,sensors.servos.at(int(ServoID::HEAD_PITCH)).presentPosition});
                headSearcher.replaceSearchPoints(fixationPoints, currentPos);
            }

            arma::vec2 HeadBehaviourSoccer::getIMUSpaceDirection(const arma::vec2& screenAngles, const Rotation3D& headToIMUSpace){               

                // arma::vec3 lookVectorFromHead = objectDirectionFromScreenAngular(screenAngles);
                arma::vec3 lookVectorFromHead = sphericalToCartesian({1,screenAngles[0],screenAngles[1]});//This is an approximation relying on the robots small FOV
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
            std::vector<arma::vec2> HeadBehaviourSoccer::getSearchPoints(std::vector<VisionObject> fixationObjects, SearchType sType){
                    //If there is nothing of interest, we search fot points of interest
                    if(fixationObjects.size() == 0){
                        //Lost searches are normalised in terms of the FOV
                        std::vector<arma::vec2> scaledResults;
                        for(auto& p : lost_searches[sType]){
                            //Interpolate between max and min allowed angles with -1 = min and 1 = max
                            scaledResults.push_back(arma::vec2({((max_yaw - min_yaw) * p[0] + max_yaw + min_yaw) / 2,
                                                                ((max_pitch - min_pitch) * p[1] + max_pitch + min_pitch) / 2}));
                        }
                        return scaledResults;
                    }
                   
                    Quad boundingBox = getScreenAngularBoundingBox(fixationObjects);
                    
                    //DEBUG
                    for(const auto& p : boundingBox.getVertices()){
                    }
                    
                    std::vector<arma::vec2> viewPoints;
                    if(arma::norm(cam.FOV) == 0){
                        log<NUClear::WARN>("NO CAMERA PARAMETERS LOADED!!");
                    }

                    //Generate search points including padding
                    //TODO: make this a generic quad method
                    //0
                    int centre = viewPoints.size();
                    viewPoints.push_back(boundingBox.getCentre());
                    //1
                    int tr = viewPoints.size();
                    arma::vec2 padding = {view_padding_radians,view_padding_radians};
                    viewPoints.push_back(boundingBox.getBottomLeft() - padding + cam.FOV / 2.0);
                    //2
                    int br = viewPoints.size();
                    padding = {view_padding_radians,-view_padding_radians};
                    viewPoints.push_back(boundingBox.getTopLeft() - padding + arma::vec({cam.FOV[0],-cam.FOV[1]}) / 2.0);
                    //3
                    int bl = viewPoints.size();
                    padding = {-view_padding_radians,-view_padding_radians};
                    viewPoints.push_back(boundingBox.getTopRight() - padding - cam.FOV / 2.0);
                    //4
                    int tl = viewPoints.size();
                    padding = {-view_padding_radians,view_padding_radians};
                    viewPoints.push_back(boundingBox.getBottomRight() - padding + arma::vec({-cam.FOV[0],cam.FOV[1]}) / 2.0);
                    
                    //Sort according to approach
                    const int nPoints = viewPoints.size();
                    int perm[nPoints];
                    switch (sType){
                        case(SearchType::LOW_FIRST):
                            perm[0] = centre; perm[1] = br; perm[2] = bl; perm[3] = tl; perm[4] = tr;
                            break;
                        case(SearchType::HIGH_FIRST):
                            perm[0] = centre; perm[1] = bl; perm[2] = br; perm[3] = tr; perm[4] = tl;
                            break;
                        case(SearchType::CROSS):
                            perm[0] = centre; perm[1] = br; perm[2] = tl; perm[3] = bl; perm[4] = tr;
                            break;
                    }
                    std::vector<arma::vec2> sortedViewPoints(nPoints);
                    for(int i = 0; i < nPoints; i++){
                        sortedViewPoints[i] = viewPoints[perm[i]];
                    }

                    return sortedViewPoints;
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

            

        }  // motion
    } //behaviour
}  // modules
