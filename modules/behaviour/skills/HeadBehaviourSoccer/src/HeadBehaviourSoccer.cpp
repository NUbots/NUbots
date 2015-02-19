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

        using utility::math::coordinates::sphericalToCartesian;
        using utility::motion::kinematics::calculateHeadJoints;
        using utility::motion::kinematics::DarwinModel;
        using utility::math::matrix::Rotation3D;
        using utility::math::geometry::Quad;

        using messages::input::ServoID;

            HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment) : 
            Reactor(std::move(environment)),
            currentWorldPitch(0),
            currentWorldYaw(0){

                //do a little configurating
                on<Trigger<Configuration<HeadBehaviourSoccer>>>("Head Behaviour Soccer Config",[this] (const Configuration<HeadBehaviourSoccer>& config)
                {
                    //Gains                    
                    p_gain_tracking = config["p_gain_tracking"].as<double>();                    
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
                    std::vector<VisionObject> fixationObjects;

                    bool search = false;

                    int ballsSeenThisUpdate = 0;
                    int goalPostsSeenThisUpdate = 0;
                    

                    int maxPriority = std::max(std::max(ballPriority,goalPriority),linePriority);


                    if(ballPriority == maxPriority){
                        if(vballs && vballs->size() > 0){
                            //Fixate on ball
                            ballsSeenThisUpdate = vballs->size();
                            auto& ball = (*vballs)[0];
                            fixationObjects.push_back(VisionObject(ball));
                        } else {
                            search = true;
                        }
                    } 
                    if(goalPriority == maxPriority){
                        if(vgoals && vgoals->size() > 0){
                            //Fixate on goals and lines and other landmarks
                            goalPostsSeenThisUpdate = vgoals->size();
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
                    
                    //TODO : Check if things have changed enough for update
                    // if(ballsSeenThisUpdate != ballsSeenLastUpdate ||
                    //    goalPostsSeenThisUpdate != goalPostsSeenLastUpdate){

                    // } else {

                    // }
                    
                    //Update
                  
                    updateHeadPlan(fixationObjects, search, sensors);
                    // ballsSeenLastUpdate = ballsSeenThisUpdate;
                    // goalPostsSeenLastUpdate = goalPostsSeenThisUpdate;
                    // lastUpdateTime = NUClear::clock::now();
                    
                    //Emit result
                    emit(getHeadCommand());
                });

              
            }

            void HeadBehaviourSoccer::updateHeadPlan(const std::vector<VisionObject>& fixationObjects, const bool& search, const Sensors& sensors){
                std::vector<arma::vec2> fixationPoints;
                std::vector<arma::vec2> fixationSizes;
                std::cout << __LINE__ << std::endl;
                arma::vec centroid = {0,0};
                std::cout << __LINE__ << std::endl;
                for(int i = 0; i < fixationObjects.size(); i++){
                    //TODO: fix arma meat errors here
                    //Should be vec2 (yaw,pitch)
                std::cout << __LINE__ << std::endl;
                    fixationPoints.push_back(arma::vec({fixationObjects[i].screenAngular[0],fixationObjects[i].screenAngular[1]}));
                std::cout << __LINE__ << std::endl;
                    fixationPoints.push_back(arma::vec({fixationObjects[i].angularSize[0],fixationObjects[i].angularSize[1]}));
                    //Average here as it is more elegant than an if statement checking if size==0 at the end
                std::cout << __LINE__ << std::endl;
                    centroid += arma::vec(fixationObjects[i].screenAngular) / (fixationObjects.size());
                }

                if(search){
                std::cout << __LINE__ << std::endl;
                    if(fixationPoints.size() > 0){
                std::cout << __LINE__ << std::endl;
                        fixationPoints = getSearchPoints(fixationPoints,fixationSizes);
                    } //else {

                    // }
                } 
                std::cout << __LINE__ << std::endl;

                //Get robot pose
                std::cout << __LINE__ << std::endl;
                Rotation3D orientation, headToBodyRotation;
                std::cout << __LINE__ << std::endl;
                if(fixationObjects.size() > 0){ 
                std::cout << __LINE__ << std::endl;
                    headToBodyRotation = fixationObjects[0].sensors->forwardKinematics.at(ServoID::HEAD_PITCH).rotation();
                std::cout << __LINE__ << std::endl;
                    orientation = fixationObjects[0].sensors->orientation.i();
                } else{
                std::cout << __LINE__ << std::endl;
                    headToBodyRotation = sensors.forwardKinematics.at(ServoID::HEAD_PITCH).rotation();
                std::cout << __LINE__ << std::endl;
                    orientation = sensors.orientation.i();
                }
                std::cout << __LINE__ << std::endl;
                arma::vec2 lookPoint = fixationPoints[0];
                //Test by looking at centroid:
                std::cout << __LINE__ << std::endl;
                arma::vec3 lookVectorFromHead = sphericalToCartesian({1,lookPoint[0],lookPoint[1]});//This is an approximation relying on the robots small FOV
                //Rotate target angles to World space
                std::cout << __LINE__ << std::endl;
                arma::vec3 lookVector =  orientation * headToBodyRotation * lookVectorFromHead;
                //Compute inverse kinematics for head direction angles
                std::cout << __LINE__ << std::endl;
                std::vector< std::pair<ServoID, float> > goalAngles = calculateHeadJoints<DarwinModel>(lookVector);
                std::cout << __LINE__ << std::endl;

                for(auto& angle : goalAngles){
                std::cout << __LINE__ << std::endl;
                    if(angle.first == ServoID::HEAD_PITCH){
                std::cout << __LINE__ << std::endl;
                        currentWorldPitch = angle.second * (p_gain_tracking) + (1 - p_gain_tracking) * currentWorldPitch;
                std::cout << __LINE__ << std::endl;
                    } else if(angle.first == ServoID::HEAD_YAW){
                std::cout << __LINE__ << std::endl;
                        currentWorldYaw = angle.second * (p_gain_tracking) + (1 - p_gain_tracking) * currentWorldYaw;
                    }
                }

            }

            std::unique_ptr<HeadCommand> HeadBehaviourSoccer::getHeadCommand(){
                return std::move(std::make_unique<HeadCommand>(HeadCommand{currentWorldYaw,currentWorldPitch}));
            }

            /*! Get search points which keep everything in view.
            Returns vector of arma::vec2 
            */
            std::vector<arma::vec2> HeadBehaviourSoccer::getSearchPoints(std::vector<arma::vec2> fixationPoints, std::vector<arma::vec2> fixationSizes){
                    //TODO: optimise? there is redundant data in these points
                std::cout << __LINE__ << std::endl;
                    std::vector<arma::vec2> boundingPoints;
                std::cout << __LINE__ << std::endl;
                    for(int i = 0; i< fixationPoints.size(); i++){
                std::cout << __LINE__ << std::endl;
                        boundingPoints.push_back(fixationPoints[i]+fixationSizes[i]);
                std::cout << __LINE__ << std::endl;
                        boundingPoints.push_back(fixationPoints[i]-fixationSizes[i]);
                    }

                std::cout << __LINE__ << std::endl;
                    Quad boundingBox = Quad::getBoundingBox(boundingPoints);
                std::cout << __LINE__ << std::endl;

                    return boundingBox.getVertices();

            }



        }  // motion
    } //behaviour
}  // modules
