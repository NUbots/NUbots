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

#include "message/behaviour/LookStrategy.h"
#include "message/behaviour/Look.h"
#include "message/behaviour/MotionCommand.h"
#include "message/behaviour/SoccerObjectPriority.h"
#include "message/support/FieldDescription.h"
#include "message/input/Sensors.h"
#include "message/motion/GetupCommand.h"
#include "message/motion/DiveCommand.h"
#include "message/localisation/FieldObject.h"
#include "message/localisation/ResetRobotHypotheses.h"
#include "message/localisation/SideChecker.h"
#include "message/support/Configuration.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/time/time.h"
#include "utility/localisation/transform.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/geometry/Circle.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace behaviour {
namespace strategy {

    using message::support::Configuration;
    using message::input::Sensors;
    using message::input::gameevents::GameState;
    using message::input::gameevents::Mode;
    using message::input::gameevents::Phase;
    using message::localisation::Ball;
    using message::localisation::Self;
    using message::localisation::SideCheckingComplete;
    using message::behaviour::MotionCommand;
    using message::behaviour::LookStrategy;
    using message::behaviour::Look;
    using message::behaviour::FieldTarget;
    using message::behaviour::KickPlan;
    using message::behaviour::KickType;
    using message::behaviour::SoccerObjectPriority;
    using message::behaviour::SearchType;
    using message::behaviour::proto::Behaviour;
    using message::support::FieldDescription;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::motion::DiveCommand;
    using message::motion::DiveFinished;
    using SelfPenalisation = message::input::gameevents::Penalisation<message::input::gameevents::SELF>;
    using SelfUnpenalisation = message::input::gameevents::Unpenalisation<message::input::gameevents::SELF>;
    using message::localisation::ResetRobotHypotheses;
    using utility::math::matrix::Transform2D;
    using message::platform::darwin::ButtonMiddleDown;
    using message::platform::darwin::ButtonLeftDown;

    using utility::localisation::transform::RobotToWorldTransform;
    using utility::time::durationFromSeconds;
    using utility::math::geometry::Circle;

    SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("SoccerStrategy.yaml").then([this](const Configuration& config) {

            cfg_.ball_last_seen_max_time = durationFromSeconds(config["ball_last_seen_max_time"].as<double>());
            cfg_.goal_last_seen_max_time = durationFromSeconds(config["goal_last_seen_max_time"].as<double>());

            cfg_.localisation_interval = durationFromSeconds(config["localisation_interval"].as<double>());
            cfg_.localisation_duration = durationFromSeconds(config["localisation_duration"].as<double>());

            cfg_.start_position_offensive = config["start_position_offensive"].as<arma::vec2>();
            cfg_.start_position_defensive = config["start_position_defensive"].as<arma::vec2>();

            cfg_.is_goalie = config["goalie"].as<bool>();

            // Use configuration here from file GoalieWalkPlanner.yaml
            cfg_.goalie_command_timeout = config["goalie_command_timeout"].as<float>();
            cfg_.goalie_rotation_speed_factor = config["goalie_rotation_speed_factor"].as<float>();
            cfg_.goalie_max_rotation_speed = config["goalie_max_rotation_speed"].as<float>();
            cfg_.goalie_translation_speed_factor = config["goalie_translation_speed_factor"].as<float>();
            cfg_.goalie_max_translation_speed = config["goalie_max_translation_speed"].as<float>();
            cfg_.goalie_side_walk_angle_threshold = config["goalie_side_walk_angle_threshold"].as<float>();

            cfg_.alwaysPowerKick = config["always_power_kick"].as<bool>();
            cfg_.forcePlaying = config["force_playing"].as<bool>();
            cfg_.forcePenaltyShootout = config["force_penalty_shootout"].as<bool>();

        });

        // TODO: unhack
        emit(std::make_unique<KickPlan>(KickPlan{{4.5, 0}, KickType::IK_KICK}));


        // For checking last seen times
        on<Trigger<std::vector<Ball>>>().then([this] (const std::vector<Ball>& balls) {
            if(!balls.empty()) {
                ballLastMeasured = balls[0].last_measurement_time;
            }
        });

        on<Trigger<std::vector<Self>>>().then([this] (const std::vector<Self>& selfs) {
            if(!selfs.empty()) {
                selfLastMeasured = selfs[0].last_measurement_time;
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

        // Check to see if we are currently in the process of diving.
        on<Trigger<DiveCommand>>().then([this] {
            isDiving = true;
        });

        // Check to see if we have finished diving.
        on<Trigger<DiveFinished>>().then([this] {
            isDiving = false;
        });

        on<Trigger<SelfPenalisation>>().then([this] {
            selfPenalised = true;
        });

        on<Trigger<SelfUnpenalisation>, With<FieldDescription>>().then([this] (const FieldDescription& fieldDescription) {
            selfPenalised = false;
            // TODO: isSideChecking = true;
            // TODO: only do this once put down
            unpenalisedLocalisationReset(fieldDescription);
        });

        on<Trigger<SideCheckingComplete>>().then([this] {
            isSideChecking = false;
        });

        on<Trigger<ButtonMiddleDown>, Single>().then([this] {

            if (!cfg_.forcePlaying) {
                NUClear::log("Force playing started.");
                cfg_.forcePlaying = true;
            }

        });

        on<Trigger<ButtonLeftDown>, Single>().then([this] {

            if (!cfg_.forcePenaltyShootout) {
                NUClear::log("Force penalty shootout started.");
                cfg_.forcePenaltyShootout = true;
            }

        });

        // Main Loop
        // TODO: ensure a reasonable state is emitted even if gamecontroller is not running
        on<Every<30, Per<std::chrono::seconds>>,
            With<Sensors>,
            With<GameState>,
            With<Phase>,
            With<FieldDescription>,
            With<std::vector<Self>>,
            With<std::vector<Ball>>,
            Single
        >().then([this](
            const Sensors& sensors,
            const GameState& gameState,
            const Phase& phase,
            const FieldDescription& fieldDescription,
            const std::vector<Self>& selfs,
            const std::vector<Ball>& balls) {

            try {

                Behaviour::State previousState = currentState;

                auto mode = cfg_.forcePenaltyShootout ? Mode::PENALTY_SHOOTOUT : gameState.mode;

                //auto& phase = gameState.phase;

                // TODO: unhack
                kickType = mode == Mode::PENALTY_SHOOTOUT || cfg_.alwaysPowerKick ? KickType::SCRIPTED : KickType::IK_KICK;

                if (cfg_.forcePlaying) {
                    play(selfs, balls, fieldDescription, mode);
                }
                else if (pickedUp(sensors)) {
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
                                walkTo(fieldDescription, cfg_.start_position_offensive);
                            }
                            else {
                                walkTo(fieldDescription, cfg_.start_position_defensive);
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
                            play(selfs, balls, fieldDescription, mode);
                        }
                    }
                }

                if (currentState != previousState) {
                    emit(std::make_unique<Behaviour::State>(currentState));
                }
            }
            catch (std::runtime_error err) {
                log(err.what());
                log("Runtime exception.");
            }
        });

        on<Trigger<std::vector<Self>>, With<FieldDescription>>().then([this] (const std::vector<Self>& selfs, const FieldDescription& fieldDescription) {
            auto kickTarget = getKickPlan(selfs, fieldDescription);
            emit(std::make_unique<KickPlan>(KickPlan{kickTarget, kickType}));
            emit(utility::nubugger::drawCircle("SocStrat_kickTarget", Circle(0.05, kickTarget), 0.123, {0.8, 0.8, 0}));
        });

    }

    void SoccerStrategy::play(const std::vector<Self>& selfs, const std::vector<Ball>& balls, const FieldDescription& fieldDescription, const Mode& mode) {
        if (penalised() && !cfg_.forcePlaying) { // penalised
            standStill();
            find({FieldTarget::SELF});
            currentState = Behaviour::PENALISED;
        }
        else if (!isSideChecking) { // not penalised
            if (cfg_.is_goalie) { // goalie
                find({FieldTarget::BALL});
                goalieWalk(selfs, balls);
                currentState = Behaviour::GOALIE_WALK;
            } else {
                if (NUClear::clock::now() - lastLocalised > cfg_.localisation_interval) {
                    standStill();
                    find({FieldTarget::BALL});
                    if (NUClear::clock::now() - lastLocalised > cfg_.localisation_interval + cfg_.localisation_duration) {
                        lastLocalised = NUClear::clock::now();
                    }
                    currentState = Behaviour::LOCALISING;
                }
                else if (NUClear::clock::now() - ballLastMeasured < cfg_.ball_last_seen_max_time) { // ball has been seen recently
                    find({FieldTarget::BALL});
                    walkTo(fieldDescription, FieldTarget::BALL);
                    currentState = Behaviour::WALK_TO_BALL;
                }
                else { // ball has not been seen recently
                    if (mode != Mode::PENALTY_SHOOTOUT && arma::norm(selfs[0].position) > 1) { // a long way away from centre
                        // walk to centre of field
                        find({FieldTarget::BALL});
                        walkTo(fieldDescription, arma::vec2({0, 0}));
                        currentState = Behaviour::MOVE_TO_CENTRE;
                    } else {
                        find({FieldTarget::BALL});
                        spinWalk();
                        currentState = Behaviour::SEARCH_FOR_BALL;
                    }
                }
            }
        }
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
        selfSideBaseLine.position = arma::vec2({2, 0});
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
        emit(std::make_unique<MotionCommand>(MotionCommand::StandStill()));
    }

    void SoccerStrategy::walkTo(const FieldDescription& fieldDescription, const FieldTarget& target) {
        if (target != FieldTarget::BALL) {
            throw std::runtime_error("SoccerStrategy::walkTo: Only FieldTarget::BALL is supported.");
        }

        arma::vec2 enemyGoal = {fieldDescription.dimensions.field_length * 0.5, 0};

        emit(std::make_unique<MotionCommand>(MotionCommand::BallApproach(enemyGoal)));
    }

    void SoccerStrategy::walkTo(const FieldDescription& fieldDescription, arma::vec position) {

        arma::vec2 enemyGoal = {fieldDescription.dimensions.field_length * 0.5, 0};

        auto goalState = Transform2D::lookAt(position, enemyGoal);
        emit(std::make_unique<MotionCommand>(MotionCommand::WalkToState(goalState)));
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

    bool SoccerStrategy::ballDistance(const Ball& ball) {
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
                case FieldTarget::SELF: {
                    soccerObjectPriority->goal = 1;
                    soccerObjectPriority->searchType = SearchType::GOAL_SEARCH;

                    break;
                }
                case FieldTarget::BALL: {
                    soccerObjectPriority->ball = 1;
                    soccerObjectPriority->searchType = SearchType::LOST;
                    break;
                }
                default:
                    throw std::runtime_error("Soccer strategy attempted to find a bad object");
            }
        }
        emit(std::move(soccerObjectPriority));

    }

    void SoccerStrategy::spinWalk() {
        emit(std::make_unique<MotionCommand>(MotionCommand::DirectCommand({0, 0, 1})));
    }

    arma::vec2 SoccerStrategy::getKickPlan(const std::vector<Self>& selfs, const message::support::FieldDescription& fieldDescription) {

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
                // Aims for behind the point that gives the shortest distance
                newTarget[0] = fieldDescription.dimensions.field_length/2 + fieldDescription.dimensions.goal_depth/2;
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

    void SoccerStrategy::goalieWalk(const std::vector<Self>& selfs, const std::vector<Ball>& balls) {
        if (!(balls.empty() || selfs.empty())) {
            std::unique_ptr<MotionCommand> motionCommand;

            float timeSinceBallSeen = std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now() - balls[0].last_measurement_time).count() * 1e-6;
            if(timeSinceBallSeen < cfg_.goalie_command_timeout){
                auto& ball = balls[0];
                auto& self = selfs[0];

                float selfBearing = std::atan2(self.heading[1], self.heading[0]);
                int signBearing = selfBearing > 0 ? 1 : -1;
                float rotationSpeed = - signBearing * std::fmin(std::fabs(cfg_.goalie_rotation_speed_factor * selfBearing), cfg_.goalie_max_rotation_speed);

                int signTranslation = ball.position[1] > 0 ? 1 : -1;
                float translationSpeed = signTranslation * std::fmin(std::fabs(cfg_.goalie_translation_speed_factor * ball.position[1]), cfg_.goalie_max_translation_speed);

                motionCommand = std::make_unique<MotionCommand>(MotionCommand::DirectCommand({0, 0, rotationSpeed}));
                if(std::fabs(selfBearing) < cfg_.goalie_side_walk_angle_threshold){
                    motionCommand->walkCommand.y() = translationSpeed;
                }
            } else {
                motionCommand = std::make_unique<MotionCommand>(MotionCommand::DirectCommand({0, 0, 0}));
            }
            emit(std::move(motionCommand));
        } else {
            emit(std::make_unique<MotionCommand>(MotionCommand::DirectCommand({0, 0, 1})));
        }

    }

} // strategy
} // behaviours
} // modules
