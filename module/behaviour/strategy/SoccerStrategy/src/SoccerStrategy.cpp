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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "SoccerStrategy.hpp"

#include <Eigen/Geometry>

#include "extension/Configuration.hpp"

#include "message/behaviour/MotionCommand.hpp"
#include "message/behaviour/Nod.hpp"
#include "message/behaviour/SoccerObjectPriority.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/ResetRobotHypotheses.hpp"
#include "message/motion/BodySide.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Goal.hpp"

#include "utility/behaviour/MotionCommand.hpp"
#include "utility/math/matrix/transform.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::behaviour::strategy {

    using extension::Configuration;

    using message::behaviour::Behaviour;
    using message::behaviour::FieldTarget;
    using message::behaviour::KickPlan;
    using KickType = message::behaviour::KickPlan::KickType;
    using message::behaviour::MotionCommand;
    using message::behaviour::Nod;
    using message::behaviour::SoccerObjectPriority;
    using SearchType = message::behaviour::SoccerObjectPriority::SearchType;
    using message::input::GameEvents;
    using message::input::GameState;
    using Phase          = message::input::GameState::Data::Phase;
    using Penalisation   = message::input::GameEvents::Penalisation;
    using Unpenalisation = message::input::GameEvents::Unpenalisation;
    using GameMode       = message::input::GameState::Data::Mode;
    using KickOffTeam    = message::input::GameEvents::KickOffTeam;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::ResetRobotHypotheses;
    using message::motion::BodySide;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::platform::ButtonLeftDown;
    using message::platform::ButtonMiddleDown;
    using message::platform::ResetRawSensors;
    using message::support::FieldDescription;
    using VisionBalls = message::vision::Balls;
    using VisionGoals = message::vision::Goals;

    using utility::support::Expression;

    using utility::support::Expression;

    SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , cfg_()
        , walkTarget()
        , lookTarget()
        , kickType()
        , ballSearchStartTime()
        , goalLastMeasured() {

        on<Configuration>("SoccerStrategy.yaml").then([this](const Configuration& config) {
            using namespace std::chrono;
            cfg_.ball_last_seen_max_time = duration_cast<NUClear::clock::duration>(
                duration<double>(config["ball_last_seen_max_time"].as<double>()));
            cfg_.goal_last_seen_max_time = duration_cast<NUClear::clock::duration>(
                duration<double>(config["goal_last_seen_max_time"].as<double>()));

            cfg_.localisation_interval =
                duration_cast<NUClear::clock::duration>(duration<double>(config["localisation_interval"].as<double>()));
            cfg_.localisation_duration =
                duration_cast<NUClear::clock::duration>(duration<double>(config["localisation_duration"].as<double>()));

            cfg_.start_position_offensive = config["start_position_offensive"].as<Expression>();
            cfg_.start_position_defensive = config["start_position_defensive"].as<Expression>();

            cfg_.is_goalie = config["goalie"].as<bool>();

            // Use configuration here from file GoalieWalkPlanner.yaml
            cfg_.goalie_command_timeout           = config["goalie_command_timeout"].as<float>();
            cfg_.goalie_rotation_speed_factor     = config["goalie_rotation_speed_factor"].as<float>();
            cfg_.goalie_max_rotation_speed        = config["goalie_max_rotation_speed"].as<float>();
            cfg_.goalie_translation_speed_factor  = config["goalie_translation_speed_factor"].as<float>();
            cfg_.goalie_max_translation_speed     = config["goalie_max_translation_speed"].as<float>();
            cfg_.goalie_side_walk_angle_threshold = config["goalie_side_walk_angle_threshold"].as<float>();

            cfg_.alwaysPowerKick      = config["always_power_kick"].as<bool>();
            cfg_.forcePlaying         = config["force_playing"].as<bool>();
            cfg_.forcePenaltyShootout = config["force_penalty_shootout"].as<bool>();
        });

        // TODO: unhack
        emit(std::make_unique<KickPlan>(KickPlan(Eigen::Vector2d(4.5, 0.0), KickType::SCRIPTED)));


        // For checking last seen times
        on<Trigger<VisionBalls>>().then([this](const VisionBalls& balls) {
            if (!balls.balls.empty()) {
                ballLastMeasured = NUClear::clock::now();
            }
        });

        on<Trigger<VisionGoals>>().then([this](const VisionGoals& goals) {
            if (!goals.goals.empty()) {
                goalLastMeasured = NUClear::clock::now();
            }
        });

        // TODO: remove this horrible code
        // Check to see if we are currently in the process of getting up.
        on<Trigger<ExecuteGetup>>().then([this] { isGettingUp = true; });

        // Check to see if we have finished getting up.
        on<Trigger<KillGetup>>().then([this] { isGettingUp = false; });

        on<Trigger<Penalisation>>().then([this](const Penalisation& selfPenalisation) {
            // Only penalise if it's us and we are not forcing playing
            if (selfPenalisation.context == GameEvents::Context::SELF && !cfg_.forcePlaying) {
                selfPenalised = true;
                oldState      = state;
                state         = State::PENALISED;
            }
        });

        on<Trigger<Unpenalisation>, With<FieldDescription>>().then(
            [this](const Unpenalisation& selfPenalisation, const FieldDescription& fieldDescription) {
                if (selfPenalisation.context == GameEvents::Context::SELF) {
                    selfPenalised = false;
                    unpenalisedLocalisationReset(fieldDescription);  // Reset localisation
                    state = oldState;  // We have been unpenalised, so lets set our state to be the old state
                    emit(std::make_unique<ResetRawSensors>());  // We have been teleported, so reset the sensors
                }
            });


        on<Trigger<ButtonMiddleDown>, Single>().then([this] {
            log("Middle button pressed!");
            if (!cfg_.forcePlaying) {
                log("Force playing started.");
                emit(std::make_unique<Nod>(true));
                cfg_.forcePlaying = true;
            }
        });

        on<Trigger<KickOffTeam>>().then(
            [this](const KickOffTeam& kickOffTeam) { team_kicking_off = kickOffTeam.context; });

        // Main Loop
        // TODO: ensure a reasonable state is emitted even if gamecontroller is not running
        on<Every<30, Per<std::chrono::seconds>>,
           With<Sensors>,
           With<GameState>,
           With<Phase>,
           With<FieldDescription>,
           With<Field>,
           With<Ball>,
           Single>()
            .then([this](const Sensors& sensors,
                         const GameState& gameState,
                         const Phase& phase,
                         const FieldDescription& fieldDescription,
                         const Field& field,
                         const Ball& ball) {
                try {
                    // Set our game mode
                    auto mode = gameState.data.mode.value;

                    // Force mode to penalty shootout if config says to
                    if (cfg_.forcePenaltyShootout) {
                        mode = GameMode::PENALTY_SHOOTOUT;
                    }

                    // Force phase to playing if config says to
                    if (cfg_.forcePlaying) {
                        phase == Phase::PLAYING;
                    }

                    // Store the current state to test later if it is changed
                    const currentStateCheck = state;

                    // Call a function based on the game mode
                    switch (mode) {
                        case GameMode::PENALTY_SHOOTOUT: penaltyShootout(phase, fieldDescription); break;
                        case GameMode::NORMAL: normal(); break;
                        case GameMode::OVERTIME: overtime(); break;
                        default: log<NUClear::WARN>("Game mode unknown.");
                    }

                    // The state has changed, update the old state
                    if (currentStateCheck != state) {
                        oldState = currentStateCheck;
                    }
                }
                catch (std::runtime_error& err) {
                    log(err.what());
                    log("Runtime exception.");
                }
            });

        on<Trigger<Field>, With<FieldDescription>>().then(
            [this](const Field& field, const FieldDescription& fieldDescription) {
                Eigen::Vector2d kickTarget = getKickPlan(field, fieldDescription);
                emit(std::make_unique<KickPlan>(KickPlan(kickTarget, kickType)));
            });
    }

    // *********** PENALTY SHOOTOUT GAME MODE *********************
    void penaltyShootout(const Phase& phase, const FieldDescription& fieldDescription) {
        // Act based on our current state
        switch (state) {
            // ******************* STATE INITIAL *****************
            case State::SHOOTOUT_INITIAL:
                // Check our current phase
                switch (phase) {
                    // If we are still in initial, just stand and don't do anything
                    case Phase::INITIAL: standStill(); break;

                    // If we are in the set phase, figure out whether we are goalie or kicker
                    case Phase::SET: state = penaltySideCheck(); break;

                    // Basic states
                    case Phase::FINISHED: state = State::FINISHED; break;
                    case Phase::TIMEOUT: state = State::TIMEOUT; break;

                    // If we were in this phase, and now are in any of these phases, then something has gone wrong
                    case Phase::READY:
                    case Phase::PLAYING: log<NUClear::WARN>("Unexpected phase."); break;
                    default: log<NUClear::WARN>("Unknown phase.");
                }
                break;
            // ***************************************************

            // ****************** STATE SET KICK *****************
            // Set state with our turn to kick
            case State::SHOOTOUT_SET_KICK:
                switch (phase) {
                    // If we are still in set phase, keep looking for the ball
                    case Phase::SET:
                        standStill();                                    // We are still in set, just be still
                        find({FieldTarget(FieldTarget::Target::BALL)});  // Look for the ball
                        break;

                    // We are now playing, so lets kick
                    case Phase::PLAYING:
                        emit(std::make_unique<MotionCommand>(utility::behaviour::PenaltyKick()));
                        state = State::SHOOTOUT_PLAYING_KICK;
                        break;

                    // Basic states
                    case Phase::FINISHED: state = State::FINISHED; break;
                    case Phase::TIMEOUT: state = State::TIMEOUT; break;

                    // If we were in this phase, and now are in any of these phases, then something has gone wrong
                    case Phase::INITIAL:
                    case Phase::READY: log<NUClear::WARN>("Unexpected phase."); break;
                    default: log<NUClear::WARN>("Unknown phase.");
                }
                break;
            // ****************************************************

            // ****************** STATE SET GOALIE *****************
            // Set state with our turn to be goalie
            case State::SHOOTOUT_SET_GOALIE:
                switch (phase) {
                    // If we are still in set phase, keep looking for the ball
                    case Phase::SET:
                        standStill();                                    // We are still in set, just be still
                        find({FieldTarget(FieldTarget::Target::BALL)});  // Look for the ball
                        break;

                    // We are now playing, lets look for the ball and keep in line with it
                    case Phase::PLAYING:
                        find({FieldTarget(FieldTarget::Target::BALL)});  // Look for the ball
                        goalieWalk(field, ball);                         // Walk to it
                        state = State::SHOOTOUT_PLAYING_GOALIE;
                        break;

                    // Basic states
                    case Phase::FINISHED: state = State::FINISHED; break;
                    case Phase::TIMEOUT: state = State::TIMEOUT; break;

                    // If we were in this phase, and now are in any of these phases, then something has gone wrong
                    case Phase::INITIAL:
                    case Phase::READY: log<NUClear::WARN>("Unexpected phase."); break;
                    default: log<NUClear::WARN>("Unknown phase.");
                }
                break;
            // ****************************************************

            // ****************** STATE PLAYING KICK *****************
            // Play state where we are kicking
            case State::SHOOTOUT_PLAYING_KICK:
                switch (phase) {
                    // If we are back in set, then we need to recheck who is kicking
                    case Phase::SET: state = penaltySideCheck(fieldDescription); break;

                    // We have already kicked and are still playing - just stand still
                    case Phase::PLAYING:
                        standStill();
                        break;

                        // Basic states
                    case Phase::FINISHED: state = State::FINISHED; break;
                    case Phase::TIMEOUT: state = State::TIMEOUT; break;

                    // If we were in this phase, and now are in any of these phases, then something has gone wrong
                    case Phase::INITIAL:
                    case Phase::READY: log<NUClear::WARN>("Unexpected phase."); break;
                    default: log<NUClear::WARN>("Unknown phase.");
                }
                break;
            // ****************************************************

            // ****************** STATE PLAYING GOALIE *****************
            // Play state where we are goalie
            case State::SHOOTOUT_PLAYING_GOALIE:
                switch (phase) {
                    // If we are back in set, then we need to recheck who is kicking
                    case Phase::SET: penaltySideCheck(fieldDescription); break;

                    // We are still playing, lets look for the ball and keep in line with it
                    case Phase::PLAYING:
                        find({FieldTarget(FieldTarget::Target::BALL)});  // Look for the ball
                        goalieWalk(field, ball);                         // Walk to it
                        break;

                    // Basic states
                    case Phase::FINISHED: state = State::FINISHED; break;
                    case Phase::TIMEOUT: state = State::TIMEOUT; break;

                    // If we were in the initial phase, and now are in any of these phases, then something has gone
                    // wrong
                    case Phase::INITIAL:
                    case Phase::READY: log<NUClear::WARN>("Unexpected phase."); break;
                    default: log<NUClear::WARN>("Unknown phase.");
                }
                break;
            // ****************************************************

            // *************** PENALISED **************************
            // We are penalised, we should do nothing
            case State::PENALISED: standStill(); break;

            // ************* FINISHED ****************************
            // The game has finished, just stop
            case State::FINISHED: standStill(); break;

            // ************** UNKNOWN ****************************
            // We should reset to the initial state if we don't know what's happening
            case State::UNKNOWN: state = SHOOTOUT_INITIAL; break;

            // ******* OTHER STATES NOT IN THIS GAME MODE - SHOULD NOT HAPPEN *********
            case State::NORMAL_INITIAL:
            case State::NORMAL_SET:
            case State::NORMAL_READY:
            case State::NORMAL_PLAYING: log<NUClear::WARN>("Unexpected state for game mode penalty shootout."); break;

            // ********* SOME UNKNOWN STATE - SHOULD NOT HAPPEN ***********
            default: log<NUClear::WARN>("Unknown state.");
        }
    }

    // ********** NORMAL GAME MODE ********************************
    void normal(const Phase& phase, const FieldDescription& fieldDescription) {
        switch (state) {
            // ********** STATE INITIAL *****************
            case State::NORMAL_INITIAL:
                switch (phase) {
                    // We are still in the initial state
                    // Stand still, reset localisation, and look for where we are (ball isn't spawned yet)
                    case Phase::INITIAL:
                        standStill();
                        find({FieldTarget(FieldTarget::Target::SELF)});
                        initialLocalisationReset(fieldDescription);
                        break;

                    // We are moving from initial to ready
                    // Lets walk to our defensive or offensive position and look for where we are (ball isn't spawned
                    // yet)
                    case Phase::READY:
                        if (gameState.data.our_kick_off) {
                            walkTo(fieldDescription, cfg_.start_position_offensive);
                        }
                        else {
                            walkTo(fieldDescription, cfg_.start_position_defensive);
                        }
                        find({FieldTarget(FieldTarget::Target::SELF)});
                        state = State::NORMAL_READY;
                        break;

                    // If we were in this phase, and now are in any of these phases, then something has gone wrong
                    case Phase::SET:
                    case Phase::PLAYING:
                    case Phase::TIMEOUT:
                    case Phase::FINISHED: log<NUClear::WARN>("Unexpected phase."); break;
                    default: log<NUClear::WARN>("Unknown phase.");
                }
                break;

            // ******************************************

            // ********** STATE READY *****************
            case State::NORMAL_READY:
                switch (phase) {
                    // We are still in the ready state
                    // Keep walking to our intended position and look for where we are
                    case Phase::READY:
                        if (gameState.data.our_kick_off) {
                            walkTo(fieldDescription, cfg_.start_position_offensive);
                        }
                        else {
                            walkTo(fieldDescription, cfg_.start_position_defensive);
                        }
                        find({FieldTarget(FieldTarget::Target::SELF)});
                        break;

                    // We are moving from the ready state to the set state
                    // Stand still and look for the ball, which should be spawning now
                    case Phase::SET:
                        standStill();
                        find({FieldTarget(FieldTarget::Target::BALL)});
                        state = State::NORMAL_SET;
                        break;

                    // If we were in this phase, and now are in any of these phases, then something has gone wrong
                    case Phase::INITIAL:
                    case Phase::PLAYING:
                    case Phase::TIMEOUT:
                    case Phase::FINISHED: log<NUClear::WARN>("Unexpected phase."); break;
                    default: log<NUClear::WARN>("Unknown phase.");
                }
                break;
            // ******************************************

            // ********** STATE SET *****************
            case State::NORMAL_SET:
                switch (phase) {
                    // We are still in the set state
                    // Stand still and look for the ball
                    case Phase::SET:
                        standStill();
                        find({FieldTarget(FieldTarget::Target::BALL)});
                        break;

                    // Now we are moving to the play state!
                    case Phase::PLAYING:
                        if (cfg_.is_goalie) {
                            state = State::NORMAL_PLAYING_GOALIE;
                        }
                        else {
                            state = State::NORMAL_PLAYING;
                        }

                        break;

                    // If we were in this phase, and now are in any of these phases, then something has gone wrong
                    case Phase::INITIAL:
                    case Phase::READY:
                    case Phase::TIMEOUT:
                    case Phase::FINISHED: log<NUClear::WARN>("Unexpected phase."); break;
                    default: log<NUClear::WARN>("Unknown phase.");
                }
                break;
            // ******************************************

            // ********** STATE PLAYING AS NON-GOALIE *****************
            case State::NORMAL_PLAYING:
                switch (phase) {
                    // We are still playing
                    // Lets check if we should update our playing state, otherwise play!
                    case Phase::PLAYING:
                        // Check if we should update to be a goalie
                        if (cfg_.is_goalie) {
                            state = State::NORMAL_PLAYING_GOALIE;
                        }
                        // Keep being a non-goalie
                        else {
                            // We have seen the ball recently, so we should walk to it
                            if (NUClear::clock::now() - ballLastMeasured < cfg_.ball_last_seen_max_time) {
                                find({FieldTarget(FieldTarget::Target::BALL)});
                                walkTo(fieldDescription, FieldTarget::Target::BALL);
                            }
                            // We have not seen the ball recently, so we should look for it
                            else {
                                Eigen::Affine2d position(field.position);
                                // We are far from the centre, so lets walk to the centre of the field
                                if (mode != GameMode::PENALTY_SHOOTOUT && (position.translation().norm() > 1)) {
                                    find({FieldTarget(FieldTarget::Target::BALL)});
                                    walkTo(fieldDescription, Eigen::Vector2d::Zero());
                                }
                                // Otherwise we are not far from the center of the field, should look for the ball
                                else {
                                    find({FieldTarget(FieldTarget::Target::BALL)});
                                    walkTo(fieldDescription, FieldTarget::Target::BALL);
                                }
                            }
                        }

                        break;

                    // If we were in this phase, and now are in any of these phases, then something has gone wrong
                    case Phase::INITIAL:
                    case Phase::SET:
                    case Phase::READY:
                    case Phase::TIMEOUT:
                    case Phase::FINISHED: log<NUClear::WARN>("Unexpected phase."); break;
                    default: log<NUClear::WARN>("Unknown phase.");
                }

                break;
            // ******************************************

            // ********** STATE PLAYING AS GOALIE *****************
            case State::NORMAL_PLAYING_GOALIE:
                switch (phase) {
                    // We are still playing
                    // Lets check if we should update our playing state, otherwise do goalie things
                    case Phase::PLAYING:
                        // Check if we should update the state
                        if (!cfg_.is_goalie) {
                            state = State::NORMAL_PLAYING;
                        }
                        // Keep being a goalie
                        else {
                            find({FieldTarget(FieldTarget::Target::BALL)});
                            goalieWalk(field, ball);
                        }

                        break;

                    // If we were in this phase, and now are in any of these phases, then something has gone wrong
                    case Phase::INITIAL:
                    case Phase::SET:
                    case Phase::READY:
                    case Phase::TIMEOUT:
                    case Phase::FINISHED: log<NUClear::WARN>("Unexpected phase."); break;
                    default: log<NUClear::WARN>("Unknown phase.");
                }

                break;
            // ******************************************


            // *************** PENALISED **************************
            case State::PENALISED:
                standStill();  // We are penalised, we should do nothing
                break;

            // ************** UNKNOWN ****************************
            // We should reset to the initial state if we don't know what's happening
            case State::UNKNOWN: state = SHOOTOUT_INITIAL; break;

            // ******* OTHER STATES NOT IN THIS GAME MODE - SHOULD NOT HAPPEN *********
            case State::SHOOTOUT_INITIAL:
            case State::SHOOTOUT_SET_KICK:
            case State::SHOOTOUT_SET_GOALIE:
            case State::SHOOTOUT_PLAYING_KICK:
            case State::SHOOTOUT_PLAYING_GOALIE: log<NUClear::WARN>("Unexpected state for game mode NORMAL."); break;

            // ********* SOME UNKNOWN STATE - SHOULD NOT HAPPEN ***********
            default: log<NUClear::WARN>("Unknown state.");
        }
    }


    // If we have just entered SET in the PENALTY_SHOOTOUT mode, we need to check if we are kicker or goalie
    State penaltySideCheck(const FieldDescription& fieldDescription) {
        State state = State::UNKNOWN;
        if (team_kicking_off == GameEvents::Context::TEAM) {
            penaltyShootoutLocalisationReset(fieldDescription);  // Reset localisation
            emit(std::make_unique<ResetRawSensors>());           // We just teleported, so reset our sensors
            standStill();                                        // We don't want to move yet
            find({FieldTarget(FieldTarget::Target::BALL)});      // Look for the ball
            state = State::SHOOTOUT_SET_KICK;                    // Set our state to set kick
        }
        // If the team kicking off is the opponent, get ready to be a goalie
        else if (team_kicking_off == GameEvents::Context::OPPONENT) {
            penaltyShootoutLocalisationReset(fieldDescription);  // Reset localisation
            emit(std::make_unique<ResetRawSensors>());           // We just teleported, so reset our sensors
            standStill();                                        // We don't want to move yet
            find({FieldTarget(FieldTarget::Target::BALL)});      // Look for the ball
            state = State::SHOOTOUT_SET_GOALIE;
        }
        return state;
    }

    void SoccerStrategy::initialLocalisationReset(const FieldDescription& fieldDescription) {

        auto reset = std::make_unique<ResetRobotHypotheses>();

        ResetRobotHypotheses::Self leftSide;
        // Start on goal line
        leftSide.position =
            Eigen::Vector2d(-fieldDescription.dimensions.field_length, fieldDescription.dimensions.field_width) * 0.5;
        leftSide.position_cov = Eigen::Vector2d::Constant(0.01).asDiagonal();
        leftSide.heading      = 0;
        leftSide.heading_var  = 0.005;

        reset->hypotheses.push_back(leftSide);
        ResetRobotHypotheses::Self rightSide;
        // Start on goal line
        rightSide.position =
            Eigen::Vector2d(-fieldDescription.dimensions.field_length, -fieldDescription.dimensions.field_width) * 0.5;
        rightSide.position_cov = Eigen::Vector2d::Constant(0.01).asDiagonal();
        rightSide.heading      = 0;
        rightSide.heading_var  = 0.005;

        reset->hypotheses.push_back(rightSide);
        emit(std::move(reset));
    }

    void SoccerStrategy::penaltyShootoutLocalisationReset(const FieldDescription& /*fieldDescription*/) {

        auto reset = std::make_unique<ResetRobotHypotheses>();

        ResetRobotHypotheses::Self selfSideBaseLine;
        selfSideBaseLine.position     = Eigen::Vector2d(2.0, 0.0);
        selfSideBaseLine.position_cov = Eigen::Vector2d::Constant(0.01).asDiagonal();
        selfSideBaseLine.heading      = 0;
        selfSideBaseLine.heading_var  = 0.005;
        reset->hypotheses.push_back(selfSideBaseLine);

        emit(std::move(reset));
    }

    void SoccerStrategy::unpenalisedLocalisationReset(const FieldDescription& fieldDescription) {

        auto reset = std::make_unique<ResetRobotHypotheses>();
        ResetRobotHypotheses::Self left;
        left.position =
            Eigen::Vector2d(-fieldDescription.penalty_robot_start, fieldDescription.dimensions.field_width * 0.5);
        left.position_cov = Eigen::Vector2d(1.0, 0.01).asDiagonal();
        left.heading      = -M_PI_2;
        left.heading_var  = 0.005;
        reset->hypotheses.push_back(left);

        ResetRobotHypotheses::Self right;
        right.position =
            Eigen::Vector2d(-fieldDescription.penalty_robot_start, -fieldDescription.dimensions.field_width * 0.5);
        right.position_cov = Eigen::Vector2d(1.0, 0.01).asDiagonal();
        right.heading      = M_PI_2;
        right.heading_var  = 0.005;
        reset->hypotheses.push_back(right);

        emit(std::move(reset));
    }

    void SoccerStrategy::searchWalk() {}

    void SoccerStrategy::standStill() {
        emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
    }

    void SoccerStrategy::walkTo(const FieldDescription& fieldDescription, const FieldTarget::Target& target) {
        if (target != FieldTarget::Target::BALL) {
            throw std::runtime_error("SoccerStrategy::walkTo: Only FieldTarget::Target::BALL is supported.");
        }

        Eigen::Vector2d enemyGoal(fieldDescription.dimensions.field_length * 0.5, 0.0);

        emit(std::make_unique<MotionCommand>(utility::behaviour::BallApproach(enemyGoal)));
    }

    void SoccerStrategy::walkTo(const FieldDescription& fieldDescription, const Eigen::Vector2d& position) {
        emit(std::make_unique<MotionCommand>(utility::behaviour::WalkToState(
            utility::math::transform::lookAt(position,
                                             Eigen::Vector2d(fieldDescription.dimensions.field_length * 0.5, 0.0)))));
    }

    bool SoccerStrategy::pickedUp(const Sensors& sensors) {
        bool feetOffGround = !sensors.feet[BodySide::LEFT].down && !sensors.feet[BodySide::RIGHT].down;
        return false && feetOffGround && !isGettingUp && sensors.Htw(2, 2) < 0.92 && sensors.Htw(2, 2) > 0.88;
    }

    bool SoccerStrategy::penalised() {
        return selfPenalised;
    }

    bool SoccerStrategy::ballDistance(const Ball& ball) {
        return ball.position.norm();
    }

    void SoccerStrategy::find(const std::vector<FieldTarget>& fieldObjects) {
        // Create the soccer object priority pointer and initialise each value to 0.
        auto soccerObjectPriority  = std::make_unique<SoccerObjectPriority>();
        soccerObjectPriority->ball = 0;
        soccerObjectPriority->goal = 0;
        soccerObjectPriority->line = 0;
        for (auto& fieldObject : fieldObjects) {
            switch (fieldObject.target.value) {
                case FieldTarget::Target::SELF: {
                    soccerObjectPriority->goal        = 1;
                    soccerObjectPriority->search_type = SearchType::GOAL_SEARCH;

                    break;
                }
                case FieldTarget::Target::BALL: {
                    soccerObjectPriority->ball        = 1;
                    soccerObjectPriority->search_type = SearchType::LOST;
                    break;
                }
                default: throw std::runtime_error("Soccer strategy attempted to find a bad object");
            }
        }
        emit(std::move(soccerObjectPriority));
    }

    void SoccerStrategy::spinWalk() {
        Eigen::Affine2d spin(Eigen::Affine2d::Identity());
        spin.linear() = Eigen::Rotation2Dd(1.0).toRotationMatrix();
        emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(spin)));
    }

    Eigen::Vector2d SoccerStrategy::getKickPlan(const Field& field, const FieldDescription& fieldDescription) {
        // Defines the box within in which the kick target is changed from the centre
        // of the oppposition goal to the perpendicular distance from the robot to the goal

        float maxKickRange =
            0.6;  // TODO: make configurable, only want to change at the last kick to avoid smart goalies
        float xTakeOverBox = maxKickRange;
        size_t error       = 0.05;
        size_t buffer      = error + 2 * fieldDescription.ball_radius;             // 15cm
        float yTakeOverBox = fieldDescription.dimensions.goal_width / 2 - buffer;  // 90-15 = 75cm
        Eigen::Affine2d position(field.position);
        float xRobot = position.translation().x();
        float yRobot = position.translation().y();
        Eigen::Vector2d newTarget;

        if ((fieldDescription.dimensions.field_length * 0.5) - xTakeOverBox < xRobot && -yTakeOverBox < yRobot
            && yRobot < yTakeOverBox) {
            // Aims for behind the point that gives the shortest distance
            newTarget.x() =
                fieldDescription.dimensions.field_length * 0.5 + fieldDescription.dimensions.goal_depth * 0.5;
            newTarget.y() = yRobot;
        }
        else {
            // Aims for the centre of the goal
            newTarget.x() = fieldDescription.dimensions.field_length * 0.5;
            newTarget.y() = 0;
        }
        return newTarget;
    }

    void SoccerStrategy::goalieWalk(const Field& field, const Ball& ball) {
        std::unique_ptr<MotionCommand> motionCommand;

        float timeSinceBallSeen =
            std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now() - ballLastMeasured).count()
            * 1e-6;
        if (timeSinceBallSeen < cfg_.goalie_command_timeout) {

            Eigen::Affine2d position(field.position);
            float fieldBearing  = Eigen::Rotation2Dd(position.rotation()).angle();
            int signBearing     = fieldBearing > 0 ? 1 : -1;
            float rotationSpeed = -signBearing
                                  * std::fmin(std::fabs(cfg_.goalie_rotation_speed_factor * fieldBearing),
                                              cfg_.goalie_max_rotation_speed);

            int signTranslation    = ball.position.y() > 0 ? 1 : -1;
            float translationSpeed = signTranslation
                                     * std::fmin(std::fabs(cfg_.goalie_translation_speed_factor * ball.position[1]),
                                                 cfg_.goalie_max_translation_speed);

            Eigen::Affine2d cmd;
            cmd.linear()      = Eigen::Rotation2Dd(rotationSpeed).matrix();
            cmd.translation() = Eigen::Vector2d::Zero();
            motionCommand     = std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(cmd));
            if (std::fabs(fieldBearing) < cfg_.goalie_side_walk_angle_threshold) {
                motionCommand->walk_command.y() = translationSpeed;
            }
        }
        else {
            motionCommand =
                std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(Eigen::Affine2d::Identity()));
        }
        emit(std::move(motionCommand));
    }
}  // namespace module::behaviour::strategy
