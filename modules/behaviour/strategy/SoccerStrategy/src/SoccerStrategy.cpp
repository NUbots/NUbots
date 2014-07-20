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
#include "messages/behaviour/WalkStrategy.h"
#include "messages/support/FieldDescription.h"
#include "messages/vision/VisionObjects.h"
#include "messages/input/Sensors.h"
#include "messages/motion/GetupCommand.h"
#include "messages/motion/DiveCommand.h"
#include "messages/localisation/FieldObject.h"
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
    using VisionBall = messages::vision::Ball;
    using VisionGoal = messages::vision::Goal;
    using LocalisationBall = messages::localisation::Ball;
    using messages::localisation::Self;
    using messages::behaviour::WalkStrategy;
    using messages::behaviour::LookStrategy;
    using messages::behaviour::WalkApproach;
    using messages::behaviour::WalkTarget;
    using messages::behaviour::FieldTarget;
    using messages::support::FieldDescription;
    using messages::motion::ExecuteGetup;
    using messages::motion::KillGetup;
    using messages::motion::DiveCommand;
    using messages::motion::DiveFinished;
    using SelfPenalisation = messages::input::gameevents::Penalisation<messages::input::gameevents::SELF>;
    using SelfUnpenalisation = messages::input::gameevents::Unpenalisation<messages::input::gameevents::SELF>;
    using utility::localisation::transform::RobotToWorldTransform;
    using utility::time::durationFromSeconds;

    SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<Configuration<SoccerStrategy>>>([this](const Configuration<SoccerStrategy>& config) {
            BALL_CLOSE_DISTANCE = config["ball_close_distance"].as<double>();
            BALL_LAST_SEEN_MAX_TIME = durationFromSeconds(config["ball_last_seen_max_time"].as<double>());
            GOAL_LAST_SEEN_MAX_TIME = durationFromSeconds(config["goal_last_seen_max_time"].as<double>());

            zone.ballActiveTimeout = durationFromSeconds(config["my_zone"]["ball_active_timeout"].as<double>());
            zone.zoneReturnTimeout = durationFromSeconds(config["my_zone"]["zone_return_timeout"].as<double>());
            zone.zone(0,0) = config["my_zone"]["zone"][0][0].as<double>();
            zone.zone(0,1) = config["my_zone"]["zone"][0][1].as<double>();
            zone.zone(1,0) = config["my_zone"]["zone"][1][0].as<double>();
            zone.zone(1,1) = config["my_zone"]["zone"][1][1].as<double>();
            zone.startPositionOffensive[0] = config["my_zone"]["start_position_offensive"][0].as<double>();
            zone.startPositionOffensive[1] = config["my_zone"]["start_position_offensive"][1].as<double>();
            zone.startPositionDefensive[0] = config["my_zone"]["start_position_defensive"][0].as<double>();
            zone.startPositionDefensive[1] = config["my_zone"]["start_position_defensive"][1].as<double>();

            zone.defaultPosition[0] = config["my_zone"]["default_position"][0].as<double>();
            zone.defaultPosition[1] = config["my_zone"]["default_position"][1].as<double>();

            zone.goalie = config["my_zone"]["goalie"].as<bool>();

        });

        // For checking last seen times
        on<Trigger<std::vector<VisionBall>>>([this] (const std::vector<VisionBall>& balls) {
            if(!balls.empty()) {
                ballLastSeen = NUClear::clock::now();
            }
        });

        on<Trigger<std::vector<VisionGoal>>>([this] (const std::vector<VisionGoal>& goals) {
            if(!goals.empty()) {
                goalLastSeen = NUClear::clock::now();
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

        on<Trigger<SelfUnpenalisation>>([this](const SelfUnpenalisation&) {
            selfPenalised = false;
        });

        // Main Loop
        on<Trigger<Every<30, Per<std::chrono::seconds>>>, With<GameState>, // TODO: ensure a reasonable state is emitted even if gamecontroller is not running
            Options<Single>>([this](const time_t&, const GameState& gameState) {


            try {

                auto& mode = gameState.mode;
                //auto& phase = gameState.phase;
                auto phase = *powerplant.get<Phase>();

                if (pickedUp()) {
                    // TODO: stand, no moving
                    standStill();
                }
                else {
                    if (mode == Mode::NORMAL || mode == Mode::OVERTIME) {
                        if (phase == Phase::INITIAL) {
                            standStill();
                            find({FieldTarget::SELF});
                        }
                        else if (phase == Phase::READY) {
                            if (gameState.ourKickOff) {
                                walkTo(zone.startPositionOffensive);
                            }
                            else {
                                walkTo(zone.startPositionDefensive);
                            }
                            find({FieldTarget::SELF});
                        }
                        else if (phase == Phase::SET) {
                            standStill();
                            find({FieldTarget::BALL});
                        }
                        else if (phase == Phase::TIMEOUT) {
                            standStill();
                            find({FieldTarget::SELF});
                        }
                        else if (phase == Phase::FINISHED) {
                            standStill();
                            find({FieldTarget::SELF});
                        }
                        else if (phase == Phase::PLAYING) {
                            if (penalised()) { // penalised
                                standStill();
                                find({FieldTarget::SELF});
                            }
                            else { // not penalised
                                if (NUClear::clock::now() - ballLastSeen < zone.ballActiveTimeout) { // ball has been seen recently
                                    if (inZone(FieldTarget::BALL) || ballDistance() <= BALL_CLOSE_DISTANCE) { // in zone and close to ball
                                        walkTo(FieldTarget::BALL);
                                        find({FieldTarget::BALL});
                                    }
                                    else { // not in zone and not close to ball
                                        if (isGoalie()) { // goalie
                                            // TODO: walkTo(zone.defaultPosition, Align::OPPOSITION_GOAL});
                                            walkTo(zone.defaultPosition);
                                            find({FieldTarget::BALL});
                                        }
                                        else { // not goalie
                                            walkTo(zone.defaultPosition);
                                            find({FieldTarget::BALL});
                                        }
                                    }
                                }
                                else { // ball has not been seen recently
                                    if (inZone(FieldTarget::SELF)) { // in own zone
                                        walkTo(zone.defaultPosition);
                                        find({FieldTarget::SELF, FieldTarget::BALL});
                                    }
                                    else { // not in zone
                                        if (NUClear::clock::now() - ballLastSeen < zone.zoneReturnTimeout) {
                                            spinWalk();
                                            find({FieldTarget::BALL});
                                        }
                                        else { // time passed since ball seen
                                            walkTo(zone.defaultPosition);
                                            find({FieldTarget::SELF, FieldTarget::BALL});
                                        }

                                    }
                                }
                            }
                        }
                    }
                    else if (mode == Mode::PENALTY_SHOOTOUT) {
                    }
                }
            }
            // catch (std::exception err) {
            catch (NUClear::metaprogramming::NoDataException err) {
                log(err.what());
                log("Exception! asdflkj");
            }
            catch (std::runtime_error err) {
                log(err.what());
                log("Exception! skldfalkjfsdjfjfjfjfjfjfj");
            }
        });
    }

    void SoccerStrategy::searchWalk() {

    }

    void SoccerStrategy::standStill() {
        auto command = std::make_unique<WalkStrategy>();
        command->walkMovementType = WalkApproach::StandStill;
        emit(std::move(command));
    }

    void SoccerStrategy::walkTo(const FieldTarget& object) {
        // TODO: find object position and call other walkTo method
        WalkTarget walkTarget;
        arma::vec2 heading;

        switch (object) {
            case FieldTarget::BALL: {
                walkTarget = WalkTarget::Ball;
                auto desc = powerplant.get<FieldDescription>();
                arma::vec enemyGoal = {desc->dimensions.field_length / 2, 0};
                heading = enemyGoal;
                break;
            }
            default:
                throw "unsupported walk target";
        }

        auto approach = std::make_unique<WalkStrategy>();
        approach->targetPositionType = walkTarget;
        approach->targetHeadingType = WalkTarget::WayPoint;
        approach->walkMovementType = WalkApproach::WalkToPoint;
        approach->heading = heading;

        emit(std::move(approach));
    }

    void SoccerStrategy::walkTo(arma::vec position) {
        auto desc = powerplant.get<FieldDescription>();
        arma::vec enemyGoal = {desc->dimensions.field_length / 2, 0};
        auto approach = std::make_unique<WalkStrategy>();
        approach->targetPositionType = WalkTarget::WayPoint;
        approach->targetHeadingType = WalkTarget::WayPoint;
        approach->walkMovementType = WalkApproach::WalkToPoint;
        approach->heading = enemyGoal;
        approach->target = position;

        emit(std::move(approach));
    }

    bool SoccerStrategy::pickedUp() {
        auto sensors = powerplant.get<Sensors>();
        bool feetOffGround = (!sensors->leftFootDown && !sensors->rightFootDown);
        return feetOffGround && !isGettingUp && !isDiving;
    }

    bool SoccerStrategy::penalised() {
        return selfPenalised;
    }

    bool SoccerStrategy::isGoalie() {
        return zone.goalie;
    }

    bool SoccerStrategy::ballDistance() {
        auto ball = powerplant.get<LocalisationBall>();
        return arma::norm(ball->position);
    }

    bool SoccerStrategy::inZone(const FieldTarget& object) {
        // TODO: check that object is in bounding box
        switch (object) {
            case FieldTarget::BALL: {
                auto self = powerplant.get<Self>();
                auto ball = powerplant.get<LocalisationBall>();
                auto position = RobotToWorldTransform(self->position, self->heading, ball->position);
                return (position[0] > zone.zone(0, 0) && position[0] < zone.zone(1, 0)
                     && position[1] < zone.zone(0, 1) && position[1] > zone.zone(1, 1));

            }
            case FieldTarget::SELF: {
                auto self = powerplant.get<Self>();
                auto position = self->position;
                return (position[0] > zone.zone(0, 0) && position[0] < zone.zone(1, 0)
                     && position[1] < zone.zone(0, 1) && position[1] > zone.zone(1, 1));
            }
            default:
                throw std::runtime_error("Soccer strategy attempted to check if an invalid object was in zone");
        }
    }

    void SoccerStrategy::find(const std::vector<FieldTarget>& fieldObjects) {
        // TODO: stop hackig this madness
        if(fieldObjects.size() == 1) {
            auto& object = fieldObjects[0];
            switch (object) {
                case FieldTarget::BALL: {
                    // Prioritise balls
                    auto strategy = std::make_unique<LookStrategy>();
                    strategy->priorities = {typeid(VisionBall)};
                    emit(std::move(strategy));
                    break;
                }
                case FieldTarget::SELF: {
                    // Prioritise goals
                    auto strategy = std::make_unique<LookStrategy>();
                    strategy->priorities = {typeid(VisionGoal)};
                    emit(std::move(strategy));
                    break;
                }
                default:
                    throw std::runtime_error("Soccer strategy attempted to find a bad object");
            }
        }
        else if(fieldObjects.size() == 2) {
            // Balls come first
            if(NUClear::clock::now() - ballLastSeen > BALL_LAST_SEEN_MAX_TIME
                || NUClear::clock::now() - goalLastSeen < GOAL_LAST_SEEN_MAX_TIME) {
                // Prioritise balls
                auto strategy = std::make_unique<LookStrategy>();
                strategy->priorities = {typeid(VisionBall)};
                emit(std::move(strategy));
            }
            else {
                // Prioritise goals
                auto strategy = std::make_unique<LookStrategy>();
                strategy->priorities = {typeid(VisionGoal)};
                emit(std::move(strategy));
            }
        }
    }

    void SoccerStrategy::spinWalk() {
        // TODO: does this work?
        auto command = std::make_unique<WalkStrategy>();
        command->walkMovementType = WalkApproach::DirectCommand;
        command->target = {0,0};
        command->heading = {1,0};
        emit(std::move(command));
    }



}  // strategy
}  // behaviours
} // modules
