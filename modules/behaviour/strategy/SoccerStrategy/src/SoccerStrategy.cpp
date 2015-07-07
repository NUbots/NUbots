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

#include "SoccerStrategy.h"

#include "messages/input/gameevents/GameEvents.h"
#include "messages/behaviour/LookStrategy.h"
#include "messages/behaviour/Look.h"
#include "messages/behaviour/WalkStrategy.h"
#include "messages/behaviour/KickPlan.h"
#include "messages/behaviour/SoccerObjectPriority.h"
#include "messages/support/FieldDescription.h"
#include "messages/input/Sensors.h"
#include "messages/motion/GetupCommand.h"
#include "messages/motion/DiveCommand.h"
#include "messages/localisation/FieldObject.h"
#include "messages/localisation/ResetRobotHypotheses.h"
#include "messages/support/Configuration.h"

#include "utility/time/time.h"
#include "utility/localisation/transform.h"

namespace modules {
namespace behaviour {
namespace strategy {

    using messages::support::Configuration;
    using messages::input::Sensors;
    using messages::input::gameevents::GameState;
    using messages::input::gameevents::Mode;
    using messages::input::gameevents::Phase;
    using LocalisationBall = messages::localisation::Ball;
    using LocalisationSelf = messages::localisation::Self;
    using messages::localisation::Self;
    using messages::behaviour::WalkStrategy;
    using messages::behaviour::LookStrategy;
    using messages::behaviour::Look;
    using messages::behaviour::WalkApproach;
    using messages::behaviour::WalkTarget;
    using messages::behaviour::FieldTarget;
    using messages::behaviour::KickPlan;
    using messages::behaviour::SoccerObjectPriority;
    using messages::behaviour::proto::Behaviour;
    using messages::support::FieldDescription;
    using messages::motion::ExecuteGetup;
    using messages::motion::KillGetup;
    using messages::motion::DiveCommand;
    using messages::motion::DiveFinished;
    using SelfPenalisation = messages::input::gameevents::Penalisation<messages::input::gameevents::SELF>;
    using SelfUnpenalisation = messages::input::gameevents::Unpenalisation<messages::input::gameevents::SELF>;
    using messages::localisation::ResetRobotHypotheses;

    using utility::localisation::transform::RobotToWorldTransform;
    using utility::time::durationFromSeconds;

    SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<Configuration<SoccerStrategy>>>([this](const Configuration<SoccerStrategy>& config) {

            BALL_CLOSE_DISTANCE = config["ball_close_distance"].as<double>();
            BALL_LAST_SEEN_MAX_TIME = durationFromSeconds(config["ball_last_seen_max_time"].as<double>());
            GOAL_LAST_SEEN_MAX_TIME = durationFromSeconds(config["goal_last_seen_max_time"].as<double>());

            START_POSITION_OFFENSIVE = config["start_position_offensive"].as<arma::vec2>();
            START_POSITION_DEFENSIVE = config["start_position_defensive"].as<arma::vec2>();

            GOALIE = config["goalie"].as<bool>();

        });

        // For checking last seen times
        on<Trigger<std::vector<LocalisationBall>>>([this] (const std::vector<LocalisationBall>& balls) {
            if(!balls.empty()) {
                ballLastMeasured = balls[0].last_measurement_time;
            }
        });

        on<Trigger<std::vector<LocalisationSelf>>>([this] (const std::vector<LocalisationSelf>& selfs) {
            if(!selfs.empty()) {
                selfLastMeasured = selfs[0].last_measurement_time;
            }
        });

        // TODO: remove this horrible code
        // Check to see if we are currently in the process of getting up.
        on<Trigger<ExecuteGetup>>([this](const ExecuteGetup&) {
            isGettingUp = true;
        });

        // Check to see if we have finished getting up.
        on<Trigger<KillGetup>>([this](const KillGetup&) {
            isGettingUp = false;
        });

        // Check to see if we are currently in the process of diving.
        on<Trigger<DiveCommand>>([this](const DiveCommand&) {
            isDiving = true;
        });

        // Check to see if we have finished diving.
        on<Trigger<DiveFinished>>([this](const DiveFinished&) {
            isDiving = false;
        });

        on<Trigger<SelfPenalisation>>([this](const SelfPenalisation&) {
            selfPenalised = true;
        });

        on<Trigger<SelfUnpenalisation>, With<FieldDescription>> ([this](const SelfUnpenalisation&, const FieldDescription& fieldDescription) {
            selfPenalised = false;
            // TODO: only do this once put down
            unpenalisedLocalisationReset(fieldDescription);
        });

        // Main Loop
        // TODO: ensure a reasonable state is emitted even if gamecontroller is not running
        on<Trigger<Every<30, Per<std::chrono::seconds>>>, With<Sensors>, With<GameState>, With<Phase>, With<FieldDescription>,
            Options<Single>>([this](const time_t&, const Sensors& sensors, const GameState& gameState, const Phase& phase, const FieldDescription& fieldDescription) {

            try {

                Behaviour::State previousState = currentState;

                auto& mode = gameState.mode;
                //auto& phase = gameState.phase;
                if (pickedUp(sensors)) {
                    // TODO: stand, no moving
                    standStill();
                    currentState = Behaviour::PICKED_UP;
                }
                else {
                    if (mode == Mode::NORMAL || mode == Mode::OVERTIME || mode == Mode::PENALTY_SHOOTOUT) {
                        if (phase == Phase::INITIAL) {
                            standStill();
                            find({FieldTarget::SELF});
                            initialLocalisationReset(fieldDescription);
                            currentState = Behaviour::INITIAL;
                        }
                        else if (phase == Phase::READY) {
                            if (gameState.ourKickOff) {
                                walkTo(fieldDescription, START_POSITION_OFFENSIVE);
                            }
                            else {
                                walkTo(fieldDescription, START_POSITION_DEFENSIVE);
                            }
                            find({FieldTarget::SELF});
                            currentState = Behaviour::READY;
                        }
                        else if (phase == Phase::SET) {
                            standStill();
                            find({FieldTarget::BALL});
                            if(mode == Mode::PENALTY_SHOOTOUT) {
                                penaltyLocalisationReset();
                            }
                            else {
                                initialLocalisationReset(fieldDescription);
                            }
                            currentState = Behaviour::SET;
                        }
                        else if (phase == Phase::TIMEOUT) {
                            standStill();
                            find({FieldTarget::SELF});
                            currentState = Behaviour::TIMEOUT;
                        }
                        else if (phase == Phase::FINISHED) {
                            standStill();
                            find({FieldTarget::SELF});
                            currentState = Behaviour::FINISHED;
                        }
                        else if (phase == Phase::PLAYING) {
                            if (penalised()) { // penalised
                                standStill();
                                find({FieldTarget::SELF});
                                currentState = Behaviour::PENALISED;
                            }
                            else { // not penalised

                                if (NUClear::clock::now() - ballLastMeasured < BALL_LAST_SEEN_MAX_TIME) { // ball has been seen recently
                                    if (!GOALIE) { // goalie
                                        walkTo(fieldDescription, FieldTarget::BALL);
                                    }
                                    find({FieldTarget::BALL});
                                    currentState = Behaviour::WALK_TO_BALL;
                                }
                                else { // ball has not been seen recently
                                    if (!GOALIE) { // goalie
                                        spinWalk();
                                    }
                                    find({FieldTarget::BALL});
                                    currentState = Behaviour::SEARCH_FOR_BALL;
                                }
                            }
                        }
                    }
                }
                

                
                if (currentState != previousState) {
                    emit(std::make_unique<Behaviour::State>(currentState));
                }
            }
            // catch (std::exception err) {
            catch (NUClear::metaprogramming::NoDataException err) {
                log(err.what());
                log("No data exception.");
            }
            catch (std::runtime_error err) {
                log(err.what());
                log("Runtime exception.");
            }
        });

    on<Trigger<std::vector<Self>>, With<FieldDescription>> ([this] (const std::vector<Self>& selfs, const FieldDescription& fieldDescription) {
        
        emit(std::make_unique<KickPlan>(KickPlan{getKickPlan(selfs, fieldDescription)}));
    
    });

    }

    void SoccerStrategy::initialLocalisationReset(const FieldDescription& fieldDescription) {

        auto reset = std::make_unique<ResetRobotHypotheses>();

        ResetRobotHypotheses::Self selfSideBaseLine;
        selfSideBaseLine.position = arma::vec2({-fieldDescription.dimensions.field_length * 0.5 + fieldDescription.dimensions.goal_area_length, 0});
        selfSideBaseLine.position_cov = arma::eye(2, 2) * 0.1;
        selfSideBaseLine.heading = 0;
        selfSideBaseLine.heading_var = 0.05;
        reset->hypotheses.push_back(selfSideBaseLine);

        emit(std::move(reset));

    }

    void SoccerStrategy::penaltyLocalisationReset() {

        auto reset = std::make_unique<ResetRobotHypotheses>();

        ResetRobotHypotheses::Self selfSideBaseLine;
        selfSideBaseLine.position = arma::vec2({1, 0});
        selfSideBaseLine.position_cov = arma::eye(2, 2) * 0.1;
        selfSideBaseLine.heading = 0;
        selfSideBaseLine.heading_var = 0.05;
        reset->hypotheses.push_back(selfSideBaseLine);

        emit(std::move(reset));

    }

    void SoccerStrategy::unpenalisedLocalisationReset(const FieldDescription& fieldDescription) {

        auto reset = std::make_unique<ResetRobotHypotheses>();
        ResetRobotHypotheses::Self selfSideLeft;
        selfSideLeft.position = arma::vec2({-fieldDescription.penalty_robot_start, fieldDescription.dimensions.field_width * 0.5});
        selfSideLeft.position_cov = arma::eye(2, 2) * 0.1;
        selfSideLeft.heading = -M_PI_2;
        selfSideLeft.heading_var = 0.05;
        reset->hypotheses.push_back(selfSideLeft);

        ResetRobotHypotheses::Self selfSideRight;
        selfSideRight.position = arma::vec2({-fieldDescription.penalty_robot_start, -fieldDescription.dimensions.field_width * 0.5});
        selfSideRight.position_cov = arma::eye(2, 2) * 0.1;
        selfSideRight.heading = M_PI_2;
        selfSideRight.heading_var = 0.05;
        reset->hypotheses.push_back(selfSideRight);

        ResetRobotHypotheses::Self selfSideBaseLine;
        selfSideBaseLine.position = arma::vec2({-fieldDescription.dimensions.field_length * 0.5 + fieldDescription.dimensions.goal_area_length, 0});
        selfSideBaseLine.position_cov = arma::eye(2, 2) * 0.1;
        selfSideBaseLine.heading = 0;
        selfSideBaseLine.heading_var = 0.05;
        reset->hypotheses.push_back(selfSideBaseLine);

        emit(std::move(reset));

    }

    void SoccerStrategy::searchWalk() {

    }

    void SoccerStrategy::standStill() {
        auto command = std::make_unique<WalkStrategy>();
        command->walkMovementType = WalkApproach::StandStill;
        emit(std::move(command));
    }

    void SoccerStrategy::walkTo(const FieldDescription& fieldDescription, const FieldTarget& object) {
        // TODO: find object position and call other walkTo method
        WalkTarget walkTarget;
        arma::vec2 heading;

        switch (object) {
            case FieldTarget::BALL: {
                walkTarget = WalkTarget::Ball;
                arma::vec2 enemyGoal({fieldDescription.dimensions.field_length * 0.5, 0});
                heading = enemyGoal;
                break;
            }
            default:
                throw std::runtime_error("unsupported walk target");
        }

        auto approach = std::make_unique<WalkStrategy>();
        approach->targetPositionType = walkTarget;
        approach->targetHeadingType = WalkTarget::WayPoint;
        approach->walkMovementType = WalkApproach::WalkToPoint;
        approach->heading = heading;

        emit(std::move(approach));

    }

    void SoccerStrategy::walkTo(const FieldDescription& fieldDescription, arma::vec position) {

        arma::vec2 enemyGoal({fieldDescription.dimensions.field_length * 0.5, 0});
        auto approach = std::make_unique<WalkStrategy>();
        approach->targetPositionType = WalkTarget::WayPoint;
        approach->targetHeadingType = WalkTarget::WayPoint;
        approach->walkMovementType = WalkApproach::WalkToPoint;
        approach->heading = enemyGoal;
        approach->target = position;

        emit(std::move(approach));

    }

    bool SoccerStrategy::pickedUp(const Sensors& sensors) {

        bool feetOffGround = !sensors.leftFootDown && !sensors.rightFootDown;
        return false
            && feetOffGround
            && !isGettingUp
            && !isDiving
            && sensors.orientation(2,2) < 0.92
            && sensors.orientation(2,2) > 0.88;

    }

    bool SoccerStrategy::penalised() {
        return selfPenalised;
    }

    bool SoccerStrategy::ballDistance(const LocalisationBall& ball) {
        return arma::norm(ball.position);
    }

    void SoccerStrategy::find(const std::vector<FieldTarget>& fieldObjects) {

        // Create the soccer object priority pointer and initialise each value to 0.
        auto soccerObjectPriority = std::make_unique<SoccerObjectPriority>();
        soccerObjectPriority->ball = 0;
        soccerObjectPriority->goal = 0;
        soccerObjectPriority->line = 0;
        for (auto& fieldObject : fieldObjects) {
            switch (fieldObject) {
                case FieldTarget::BALL: {
                    soccerObjectPriority->ball = 1;
                    break;
                }
                case FieldTarget::SELF: {
                    soccerObjectPriority->goal = 1;
                    break;
                }
                default:
                    throw std::runtime_error("Soccer strategy attempted to find a bad object");
            }
        }
        emit(std::move(soccerObjectPriority));

    }

    void SoccerStrategy::spinWalk() {
        // TODO: does this work?
        auto command = std::make_unique<WalkStrategy>();
        command->walkMovementType = WalkApproach::DirectCommand;
        command->target = {0,0};
        command->heading = {1,0};
        emit(std::move(command));
    }

    arma::vec2 SoccerStrategy::getKickPlan(const std::vector<Self>& selfs, const messages::support::FieldDescription& fieldDescription) {
        
        // Defines the box within in which the kick target is changed from the centre 
        // of the oppposition goal to the perpendicular distance from the robot to the goal

        float maxKickRange = 0.6; //TODO: make configurable, only want to change at the last kick to avoid smart goalies
        float xTakeOverBox = maxKickRange;
        size_t error = 0.05;
        size_t buffer = error + 2 * fieldDescription.ball_radius; //15cm           
        float yTakeOverBox = fieldDescription.dimensions.goal_width/2 - buffer; // 90-15 = 75cm
        float xRobot = selfs.front().position[0];
        float yRobot = selfs.front().position[1];
        arma::vec2 newTarget;
        
        if(!selfs.empty()) {        
            
            if( (fieldDescription.dimensions.field_length/2) - xTakeOverBox < xRobot 
                    && -yTakeOverBox < yRobot 
                        && yRobot < yTakeOverBox) {
                
                // Aims for the point that gives the shortest distance                   
                newTarget[0] = fieldDescription.dimensions.field_length/2;
                newTarget[1] = yRobot;

            } else {
                
                // Aims for the centre of the goal
                newTarget[0] = fieldDescription.dimensions.field_length/2;
                newTarget[1] = 0;
            }

        } else {
            
            // Return default
            newTarget[0] = fieldDescription.dimensions.field_length/2;
            newTarget[1] = 0;

        }
        return newTarget;
    }

} // strategy
} // behaviours
} // modules
