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

#include "LookPlanner.h"

#include "messages/behaviour/LookStrategy.h"
#include "messages/behaviour/Look.h"
#include "messages/vision/VisionObjects.h"
#include "messages/localisation/FieldObject.h"
#include "messages/support/FieldDescription.h"
#include "messages/support/Configuration.h"
#include "utility/localisation/transform.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/time/time.h"
#include "utility/support/armayamlconversions.h"

namespace modules {
namespace behaviour {
namespace planning {

    using messages::behaviour::LookStrategy;

    using VisionBall = messages::vision::Ball;
    using VisionGoal = messages::vision::Goal;

    using messages::input::Sensors;
    using LocalisationBall = messages::localisation::Ball;
    using messages::localisation::Self;
    using messages::support::FieldDescription;
    using messages::support::Configuration;
    using utility::motion::kinematics::calculateHeadJointPosition;
    using utility::localisation::transform::RobotToWorldTransform;
    using utility::localisation::transform::WorldToRobotTransform;
    using messages::behaviour::Look;

    LookPlanner::LookPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<LookPlanner>>>([this] (const Configuration<LookPlanner>& config){
            //pan speeds
            VISUAL_TRACKING_TIMEOUT = config["visual_tracking_timeout"].as<double>();
            LOCALISATION_TRACKING_TIMEOUT = config["localisation_tracking_timeout"].as<double>();
            lostPanPoints.clear();
            for (uint i = 0; i < config["lost_pan_points"].size(); i++) {
                lostPanPoints.push_back(config["lost_pan_points"][i].as<arma::vec>());
            }
        });


        on<Trigger<Last<5, std::vector<VisionBall>>>,
            With<Optional<LocalisationBall>>,
            With<Optional<std::vector<Self>>>,
            With<Sensors>,
            With<LookStrategy>,
            With<FieldDescription>,
            Options<Sync<LookPlanner>>>
            ([this] (const LastList<std::vector<VisionBall>>& v,
                    const std::shared_ptr<const LocalisationBall>& l,
                    const std::shared_ptr<const std::vector<Self>>& selfs,
                    const Sensors& sensors,
                    const LookStrategy& strat,
                    const FieldDescription fieldDesc) {

            std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;

            //update the time last seen
            if (!v.back()->empty()) {
                timeBallSeen = NUClear::clock::now();
            }

            //find a ball with valid sensors
            auto vball = std::find_if(v.rbegin(), v.rend(), [] (const std::shared_ptr<const std::vector<VisionBall>>& a) {
                return a->empty();
            });

            ballObjects.clear();
            //check if the ball is visible
            if (vball != v.rend() and !(*vball)->empty()) {
                ballObjects.push_back({(*vball)->at(0).screenAngular, (*vball)->at(0).angularSize});
            } else {
                //check if the localisation ball is valid
                if (l != NULL) {
                    //push an object for the localisation ball
                    double ballDiameter = 2.0*atan2(fieldDesc.ball_radius,arma::norm(l->position));

                    ballObjects.push_back({utility::motion::kinematics::calculateHeadJointsToLookAt(
                                                                {l->position[0], l->position[1], 0},
                                                                sensors.orientationCamToGround,
                                                                sensors.orientationBodyToGround),
                                           arma::vec2({ballDiameter,ballDiameter}) });

                    //update the pan configuration for the lost ball pan (stored in global/field coordinates at this point)
                    ballPanPoints.clear();

                    //get global pos
                    const arma::vec2 worldBall =  utility::localisation::transform::RobotToWorldTransform(selfs->front().position,
                                                                                                          selfs->front().heading,
                                                                                                          l->position);

                    //get global standard deviation
                    const arma::vec2 worldSTD = arma::sqrt(
                                                    arma::abs(
                                                        RobotToWorldTransform(
                                                            selfs->front().position,
                                                            selfs->front().heading,
                                                            arma::vec2({l->position_cov(0,0),l->position_cov(1,1)})
                                                        )
                                                    )
                                                );

                    //push 4 surrounding pan points
                    ballPanPoints.push_back(worldBall + arma::vec2({worldSTD[0],0.0}));
                    ballPanPoints.push_back(worldBall + arma::vec2({0.0,worldSTD[1]}));
                    ballPanPoints.push_back(worldBall + arma::vec2({-worldSTD[0],0.0}));
                    ballPanPoints.push_back(worldBall + arma::vec2({0.0,-worldSTD[1]}));
                }

                //XXX: use other robots' balls if enabled
            }

            //XXX: update the look-plan
            (void)v;
            (void)l;
            (void)strat;

            std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
        });

        on<Trigger<Last<5, std::vector<VisionGoal>>>,
            With<Optional<std::vector<Self>>>,
            With<Sensors>,
            With<LookStrategy>,
            With<FieldDescription>,
            Options<Sync<LookPlanner>>>
            ([this] (const LastList<std::vector<VisionGoal>>& v,
                     const std::shared_ptr<const std::vector<Self>>& selfs,
                     const Sensors& sensors,
                     const LookStrategy& strat,
                     const FieldDescription fieldDesc) {

            std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;

            //update the time last seen
            if (!v.back()->empty()) {
                timeGoalSeen = NUClear::clock::now();
            }

            //find a ball with valid sensors
            auto vgoal = std::find_if(v.rbegin(), v.rend(), [] (const std::shared_ptr<const std::vector<VisionGoal>>& a) {
                return a->empty();
            });


            goalObjects.clear();
            //check if the ball is visible
            if (vgoal != v.rend() and !(*vgoal)->empty()) {
                goalObjects.push_back({(**vgoal)[0].screenAngular, (**vgoal)[0].angularSize});
            } else if (selfs != NULL) {
                //push an object for the localisation ball


                std::vector<arma::vec2> robotGoals;
                robotGoals.push_back(WorldToRobotTransform(selfs->front().position,
                                                           selfs->front().heading,
                                                           fieldDesc.goalpost_bl));
                robotGoals.push_back(WorldToRobotTransform(selfs->front().position,
                                                           selfs->front().heading,
                                                           fieldDesc.goalpost_br));
                robotGoals.push_back(WorldToRobotTransform(selfs->front().position,
                                                           selfs->front().heading,
                                                           fieldDesc.goalpost_yl));
                robotGoals.push_back(WorldToRobotTransform(selfs->front().position,
                                                           selfs->front().heading,
                                                           fieldDesc.goalpost_yr));
                for (const auto& g : robotGoals) {
                    double goalDiameter = 2.0*atan2(fieldDesc.dimensions.goalpost_diameter,arma::norm(g));
                    goalObjects.push_back({utility::motion::kinematics::calculateHeadJointsToLookAt(
                                                            {g[0], g[1], 0},
                                                            sensors.orientationCamToGround,
                                                            sensors.orientationBodyToGround),
                                       arma::vec2({goalDiameter,goalDiameter})});
                }

                //update the pan configuration for the lost ball pan (stored in global/field coordinates at this point)
                goalPanPoints.clear();

                //get global standard deviation
                const arma::vec2 worldSTD = arma::sqrt(arma::abs(arma::vec2({selfs->front().position_cov(0,0),selfs->front().position_cov(1,1)})));

                //get global pos
                arma::vec2 leftGoal,rightGoal;
                if (selfs->front().heading[0] > 0.0) {
                    leftGoal = fieldDesc.goalpost_yl;
                    rightGoal = fieldDesc.goalpost_yr;

                    //push 4 surrounding pan points
                    goalPanPoints.push_back((leftGoal+rightGoal)*0.5 + arma::vec2({worldSTD[0],0.0}));
                    goalPanPoints.push_back(leftGoal + arma::vec2({0.0,worldSTD[1]}));
                    goalPanPoints.push_back((leftGoal+rightGoal)*0.5 + arma::vec2({-worldSTD[0],0.0}));
                    goalPanPoints.push_back(rightGoal + arma::vec2({0.0,-worldSTD[1]}));

                } else {
                    leftGoal = fieldDesc.goalpost_bl;
                    rightGoal = fieldDesc.goalpost_br;

                    //push 4 surrounding pan points
                    goalPanPoints.push_back((leftGoal+rightGoal)*0.5 + arma::vec2({worldSTD[0],0.0}));
                    goalPanPoints.push_back(rightGoal + arma::vec2({0.0,worldSTD[1]}));
                    goalPanPoints.push_back((leftGoal+rightGoal)*0.5 + arma::vec2({-worldSTD[0],0.0}));
                    goalPanPoints.push_back(leftGoal + arma::vec2({0.0,-worldSTD[1]}));

                }

                //XXX: use other robots' detected players if enabled
            }

            std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
            updateLookPlan(strat);
            std::cout<<__FILE__<<", "<<__LINE__<<": "<< std::endl;
        });
    }


    void LookPlanner::updateLookPlan(const LookStrategy& strat) {

        //find what our timeout timer is at
        double timeDiff;
        if (strat.priorities.front() == typeid(VisionBall)) {
            timeDiff = utility::time::TimeDifferenceSeconds(NUClear::clock::now(),timeBallSeen);
        } else {
            timeDiff = utility::time::TimeDifferenceSeconds(NUClear::clock::now(),timeGoalSeen);
        }


        //do visual tracking if the object was seen recently
        if (timeDiff < VISUAL_TRACKING_TIMEOUT) {
            auto result = std::make_unique<std::vector<Look::Fixation>>();
            result->reserve(ballObjects.size()+goalObjects.size());

            if (strat.priorities.front() == typeid(VisionBall)) { //check which objects to look at first
                for (const auto& i : ballObjects) {
                    result->push_back({i.first,i.second});
                }
                for (const auto& i : goalObjects) {
                    result->push_back({i.first,i.second});
                }
            } else {
                for (const auto& i : goalObjects) {
                    result->push_back({i.first,i.second});
                }
                for (const auto& i : ballObjects) {
                    result->push_back({i.first,i.second});
                }
            }
            emit(std::move(result));

        //do localisation tracking if we haven't seen anything recently
        } else if (timeDiff < LOCALISATION_TRACKING_TIMEOUT) {
            auto result = std::make_unique<std::vector<Look::Pan>>();

            if (strat.priorities.front() == typeid(VisionBall)) { //check which objects to pan for
                result->reserve(ballPanPoints.size());
                for (const auto& i : ballPanPoints) {
                    result->push_back({i,arma::vec2({0.0,0.0})});
                }
            } else {
                result->reserve(goalPanPoints.size());
                for (const auto& i : goalPanPoints) {
                    result->push_back({i,arma::vec2({0.0,0.0})});
                }
            }
            emit(std::move(result));

        //do a pan'n'scan if things have been missing for a while
        } else {
            auto result = std::make_unique<std::vector<Look::Pan>>();

            result->reserve(lostPanPoints.size());
            for (const auto& i : lostPanPoints) {
                result->push_back({i,arma::vec2({0.0,0.0})});
            }
            emit(std::move(result));
        }
    }
}
}
}
