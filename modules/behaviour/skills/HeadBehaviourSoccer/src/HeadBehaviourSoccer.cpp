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
#include "messages/input/Sensors.h"
#include "messages/motion/HeadCommand.h"
#include "utility/math/coordinates.h"
#include "utility/motion/InverseKinematics.h"


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

            HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment) : 
            Reactor(std::move(environment)){

                //do a little configurating
                // on<Trigger<Configuration<HeadBehaviourSoccer>>>([this] (const Configuration<HeadBehaviourSoccer>& config)
                // {
                //     //Gains                    
                //     //head_gain = config["head_gain"].as<double>();
                //     
                // });

                on<
                    Trigger<Every<30, Per<std::chrono::seconds>>>,
                    With<Sensors>,
                    With<Optional<std::vector<Ball>>>,
                    With<Optional<std::vector<Goal>>>
                  >([this] (const time_t&,
                            const Sensors& sensors,
                            const std::shared_ptr<const std::vector<Ball>>& vballs,
                            const std::shared_ptr<const std::vector<Goal>>& vgoals
                            ) {
                    //Input
                    int ballPriority = 1;
                    int goalPriority = 1;
                    int linePriority = 0;

                    //Output
                    std::vector<std::unique_ptr<VisionObject>> fixationObjects;

                    bool search = false;

                    int ballsSeenThisUpdate = 0;
                    int goalPostsSeenThisUpdate = 0;
                    

                    int maxPriority = std::max(std::max(ballPriority,goalPriority),linePriority);


                    if(ballPriority == maxPriority){
                        if(vballs){
                            //Fixate on ball
                            ballsSeenThisUpdate = vballs->size();
                            auto& ball = (*vballs)[0];
                            fixationObjects.push_back(std::make_unique<VisionObject>(ball));
                        } else {
                            search = true;
                        }
                    } 
                    if(goalPriority == maxPriority){
                        if(vgoals){
                            //Fixate on goals and lines and other landmarks
                            goalPostsSeenThisUpdate = vgoals->size();
                            std::set<Goal::Side> visiblePosts;
                            for (auto& goal : (*vgoals)){
                                visiblePosts.insert(goal.side);
                                fixationObjects.push_back(std::make_unique<VisionObject>(goal));
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
                  
                    updateHeadPlan(fixationObjects, search);
                    // ballsSeenLastUpdate = ballsSeenThisUpdate;
                    // goalPostsSeenLastUpdate = goalPostsSeenThisUpdate;
                    // lastUpdateTime = NUClear::clock::now();
                    
                    //Emit result
                    emit(getHeadCommand());
                });

              
            }

            void HeadBehaviourSoccer::updateHeadPlan(const std::vector<std::unique_ptr<VisionObject>>& fixationObjects, const bool& search){
                std::vector<arma::vec2> fixationPoints;
                std::vector<arma::vec2> fixationSizes;
                arma::vec2 centroid;
                for(int i = 0; i < fixationObjects.size(); i++){
                    fixationPoints.push_back(fixationObjects[i]->screenAngular);
                    fixationPoints.push_back(fixationObjects[i]->angularSize);
                    centroid += fixationObjects[i]->screenAngular;
                }
                centroid = centroid / float(fixationObjects.size());

                //Test by looking at centroid:
                //arma::vec3 lookVectorFromHead = sphericalToCartesian();//This is an approximation relying on the robots small FOV

            }

            std::unique_ptr<HeadCommand> HeadBehaviourSoccer::getHeadCommand(){
                return std::move(std::make_unique<HeadCommand>());
            }



        }  // motion
    } //behaviour
}  // modules
