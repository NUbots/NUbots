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
#include "utility/support/yaml_armadillo.h"



namespace modules {
    namespace behaviour{
        namespace skills {

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

        using messages::input::ServoID;

            HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment) : 
            Reactor(std::move(environment)),
            lastCentroid({0,0})
            {
                //do a little configurating
                on<Trigger<Configuration<HeadBehaviourSoccer>>>("Head Behaviour Soccer Config",[this] (const Configuration<HeadBehaviourSoccer>& config)
                {
                    lastPlanUpdate = NUClear::clock::now();
                    timeLastObjectSeen = NUClear::clock::now();
                    //Gains                    
                    view_padding_radians = config["view_padding_radians"].as<double>();      

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
                });

                on<Trigger<CameraParameters>>("Head Behaviour - Load CameraParameters",[this] (const CameraParameters& cam_){
                    cam = cam_;
                });

                //TODO: trigger on balls with goals and check number of balls.
                on<
                    Trigger<Every<30, Per<std::chrono::seconds>>>,
                    With<Sensors>,
                    With<Optional<std::vector<Ball>>>,
                    With<Optional<std::vector<Goal>>>,
                    Options<Single>
                  >("Head Behaviour Main Loop",[this] ( const time_t&,
                                                        const Sensors& sensors,
                                                        const std::shared_ptr<const std::vector<Ball>>& vballs,
                                                        const std::shared_ptr<const std::vector<Goal>>& vgoals
                                                        ) {
                    //Input
                    int ballPriority = 1;
                    int goalPriority = 1;
                    int linePriority = 0;

                    //Output

                        std::cout << __LINE__ << std::endl;
                    bool search = false;

                    int maxPriority = std::max(std::max(ballPriority,goalPriority),linePriority);

                    std::vector<VisionObject> fixationObjects;

                        std::cout << __LINE__ << std::endl;
                    auto now = NUClear::clock::now();
                    //TODO: make this a loop over a list of objects or something
                        std::cout << __LINE__ << std::endl;
                    if(ballPriority == maxPriority){
                        if(vballs && vballs->size() > 0){
                            //Fixate on ball
                            timeLastObjectSeen = now;
                            auto& ball = (*vballs)[0];
                            fixationObjects.push_back(VisionObject(ball));
                        } else {
                            search = true;
                        }
                    } 
                        std::cout << __LINE__ << std::endl;
                    if(goalPriority == maxPriority){
                        if(vgoals && vgoals->size() > 0){
                            //Fixate on goals and lines and other landmarks
                            timeLastObjectSeen = now;
                            std::set<Goal::Side> visiblePosts;
                            for (auto& goal : (*vgoals)){
                                visiblePosts.insert(goal.side);
                                fixationObjects.push_back(VisionObject(goal));
                            }
                            search = (visiblePosts.find(Goal::Side::LEFT) == visiblePosts.end() ||//If left post not visible or
                                      visiblePosts.find(Goal::Side::RIGHT) == visiblePosts.end());//right post not visible, then we need to search for the other goal post
                        } else {
                            search = true;
                        }
                    }
                    //Do we need to update our plan?
                    bool updatePlan = false;
                    bool lost = fixationObjects.size() == 0;

                        std::cout << __LINE__ << std::endl;

                    //Get robot pose
                    Rotation3D orientation, headToBodyRotation;
                    if(!lost){
                        //We need to transform our view points to orientation space
                        headToBodyRotation = fixationObjects[0].sensors->forwardKinematics.at(ServoID::HEAD_PITCH).rotation();
                        orientation = fixationObjects[0].sensors->orientation.i();
                    } else {
                        headToBodyRotation = arma::eye(3,3);
                        orientation = sensors.orientation.i();
                    }
                    Rotation3D headToIMUSpace = orientation * headToBodyRotation;                    
                        std::cout << __LINE__ << std::endl;

                    //Check current centroid
                    if(fixationObjects.size() > 0){
                        arma::vec2 currentCentroid = arma::vec2({0,0});
                        std::cout << __LINE__ << std::endl;
                        for(auto& ob : fixationObjects){
                        std::cout << __LINE__ << std::endl;
                            currentCentroid = ob.screenAngular / float(fixationObjects.size());
                        }
                        std::cout << __LINE__ << std::endl;
                        currentCentroid = getIMUSpaceDirection(currentCentroid,headToIMUSpace);
                        std::cout << __LINE__ << std::endl;
                        if(arma::norm(currentCentroid - lastCentroid) > angular_update_threshold * std::fmax(cam.FOV[0],cam.FOV[1]) / 2.0){
                            updatePlan = true;
                        std::cout << __LINE__ << std::endl;
                            lastCentroid = currentCentroid;
                        std::cout << __LINE__ << std::endl;
                        }
                        std::cout << __LINE__ << std::endl;
                    }

                        std::cout << __LINE__ << std::endl;

                    //If we lost what we are searching for
                    updatePlan = updatePlan || std::chrono::duration_cast<std::chrono::milliseconds>(now - timeLastObjectSeen).count() > angular_update_threshold;

                        std::cout << __LINE__ << std::endl;
                    if(updatePlan){
                        std::cout << "UpdatingPlan:" << std::endl;
                        updateHeadPlan(fixationObjects, search, sensors);
                    }

                        std::cout << __LINE__ << std::endl;
                    //Update state machine
                    headSearcher.update();
                    //Emit new result if possible
                        std::cout << __LINE__ << std::endl;
                    if(headSearcher.newGoal()){
                        //Emit result
                        std::cout << "Emmitting new head command:" << std::endl;                        
                        arma::vec2 direction = getIMUSpaceDirection(headSearcher.getState(), headToIMUSpace);
                        std::cout << __LINE__ << std::endl;
                        std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                        std::cout << __LINE__ << std::endl;
                        command->yaw = direction[0];
                        std::cout << __LINE__ << std::endl;
                        command->pitch = direction[1];
                        std::cout << __LINE__ << std::endl;
                        emit(std::move(command));
                    }
                });

              
            }

            void HeadBehaviourSoccer::updateHeadPlan(const std::vector<VisionObject>& fixationObjects, const bool& search, const Sensors& sensors){
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
                    fixationPoints = getSearchPoints(fixationPoints, fixationSizes, SearchType::LOW_FIRST);
                }

                if(fixationPoints.size() <= 0){
                    log("FOUND NO POINTS TO LOOK AT! - ARE THE SEARCHES PROPERLY CONFIGURED IN HEADBEHAVIOURSOCCER.YAML?");
                }
                
                auto currentPos = arma::vec2({sensors.servos.at(int(ServoID::HEAD_YAW)).presentPosition,sensors.servos.at(int(ServoID::HEAD_PITCH)).presentPosition});
                headSearcher.replaceSearchPoints(fixationPoints, currentPos);
            }

            arma::vec2 HeadBehaviourSoccer::getIMUSpaceDirection(const arma::vec2& lookPoint, const Rotation3D& headToIMUSpace){               

                arma::vec3 lookVectorFromHead = sphericalToCartesian({1,lookPoint[0],lookPoint[1]});//This is an approximation relying on the robots small FOV
                //Rotate target angles to World space
                arma::vec3 lookVector =  headToIMUSpace * lookVectorFromHead;
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
            std::vector<arma::vec2> HeadBehaviourSoccer::getSearchPoints(std::vector<arma::vec2> fixationPoints, std::vector<arma::vec2> fixationSizes, SearchType sType){
                    //If there is nothing of interest, we search fot points of interest
                    if(fixationPoints.size() == 0){
                        //Lost searches are normalised in terms of the FOV
                        std::vector<arma::vec2> scaledResults;
                        for(auto& p : lost_searches[sType]){
                            //Interpolate between max and min allowed angles with -1 = min and 1 = max
                            scaledResults.push_back(arma::vec2({((max_yaw - min_yaw) * p[0] + max_yaw + min_yaw) / 2,
                                                                ((max_pitch - min_pitch) * p[1] + max_pitch + min_pitch) / 2}));
                        }
                        return scaledResults;
                    }
                    //TODO: optimise? there is redundant data in these points
                    std::vector<arma::vec2> boundingPoints;
                    for(uint i = 0; i< fixationPoints.size(); i++){
                        boundingPoints.push_back(fixationPoints[i]+fixationSizes[i] / 2);
                        boundingPoints.push_back(fixationPoints[i]-fixationSizes[i] / 2);
                    }

                    Quad boundingBox = Quad::getBoundingBox(boundingPoints);

                    std::vector<arma::vec2> viewPoints;
                    if(arma::norm(cam.FOV) == 0){
                        log<NUClear::WARN>("NO CAMERA PARAMETERS LOADED!!");
                    }

                    //Generate search points including padding
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

            

        }  // motion
    } //behaviour
}  // modules
