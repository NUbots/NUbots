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
#include "message/input/GameEvents.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/ResetBallHypotheses.hpp"
#include "message/localisation/ResetRobotHypotheses.hpp"
#include "message/motion/BodySide.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/KickCommand.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Goal.hpp"

#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"
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
    using KickOffTeam    = message::input::GameEvents::KickOffTeam;
    using Penalisation   = message::input::GameEvents::Penalisation;
    using Unpenalisation = message::input::GameEvents::Unpenalisation;
    using GameMode       = message::input::GameState::Data::Mode;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::ResetBallHypotheses;
    using message::localisation::ResetRobotHypotheses;
    using message::motion::BodySide;
    using message::motion::ExecuteGetup;
    using message::motion::KickCommandType;
    using message::motion::KickScriptCommand;
    using message::motion::KillGetup;
    using message::platform::ButtonMiddleDown;
    using message::platform::ResetWebotsServos;
    using message::support::FieldDescription;
    using VisionBalls = message::vision::Balls;
    using VisionGoals = message::vision::Goals;

    using utility::input::LimbID;
    using utility::support::Expression;

    using utility::support::Expression;

    SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("SoccerStrategy.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

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

            // Use configuration here from file SoccerStrategy.yaml
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

        // TODO(BehaviourTeam): unhack
        emit(std::make_unique<KickPlan>(KickPlan(Eigen::Vector2d(4.5, 0.0), KickType::SCRIPTED)));

        on<Trigger<Field>, With<FieldDescription>>().then(
            [this](const Field& field, const FieldDescription& fieldDescription) {
                Eigen::Vector2d kickTarget = getKickPlan(field, fieldDescription);
                emit(std::make_unique<KickPlan>(KickPlan(kickTarget, kickType)));
            });

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

        // TODO(BehaviourTeam): remove this horrible code
        // Check to see if we are currently in the process of getting up.
        on<Trigger<ExecuteGetup>>().then([this] { isGettingUp = true; });

        // Check to see if we have finished getting up.
        on<Trigger<KillGetup>>().then([this] { isGettingUp = false; });

        on<Trigger<Penalisation>>().then([this](const Penalisation& selfPenalisation) {
            if (selfPenalisation.context == GameEvents::Context::SELF) {
                emit(std::make_unique<ResetWebotsServos>());
                selfPenalised = true;
            }
        });

        on<Trigger<Unpenalisation>>().then([this](const Unpenalisation& selfPenalisation) {
            if (selfPenalisation.context == GameEvents::Context::SELF) {
                selfPenalised = false;

                // TODO(BehaviourTeam): isSideChecking = true;
                // TODO(BehaviourTeam): only do this once put down
                unpenalisedLocalisationReset();
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

        // ********************* Main Loop ********************
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

                    // Force penalty shootout mode if set in config
                    auto mode = cfg_.forcePenaltyShootout ? GameMode::PENALTY_SHOOTOUT : gameState.data.mode.value;

                    kickType = KickType::SCRIPTED;  // Use the scripted kick

                    Behaviour::State previousState = currentState;  // Update previous state

                    // If force playing, just run the play functions based on mode
                    if (cfg_.forcePlaying) {
                        if (mode == GameMode::PENALTY_SHOOTOUT) {
                            penaltyShootoutPlaying(field, ball);
                        }
                        else {
                            normalPlaying(field, ball, fieldDescription);
                        }
                    }
                    // If we're picked up, stand still
                    else if (pickedUp(sensors)) {
                        // TODO(BehaviourTeam): stand, no moving
                        standStill();
                        currentState = Behaviour::State::PICKED_UP;
                    }

                    // Switch gamemode statemachine based on GameController state
                    switch (mode) {
                        case GameMode::PENALTY_SHOOTOUT: penaltyShootout(phase, fieldDescription, field, ball); break;
                        // We handle NORMAL and OVERTIME the same at the moment because we don't have any special
                        // behaviour for overtime.
                        case GameMode::NORMAL:
                        case GameMode::OVERTIME: normal(gameState, phase, fieldDescription, field, ball); break;
                        default: log<NUClear::WARN>("Game mode unknown.");
                    }

                    if (currentState != previousState) {
                        emit(std::make_unique<Behaviour::State>(currentState));
                    }
                }
                catch (std::runtime_error& err) {
                    log(err.what());
                    log("Runtime exception.");
                }
            });
    }

    // ********************PENALTY GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::penaltyShootout(const Phase& phase,
                                         const FieldDescription& fieldDescription,
                                         const Field& field,
                                         const Ball& ball) {
        switch (phase.value) {
            case Phase::INITIAL: penaltyShootoutInitial(); break;             // Happens at beginning.
            case Phase::READY: penaltyShootoutReady(); break;                 // Should not happen.
            case Phase::SET: penaltyShootoutSet(fieldDescription); break;     // Happens on beginning of each kick try.
            case Phase::PLAYING: penaltyShootoutPlaying(field, ball); break;  // Either kicking or goalie.
            case Phase::TIMEOUT: penaltyShootoutTimeout(); break;             // A pause in playing. Not in simulation.
            case Phase::FINISHED: penaltyShootoutFinished(); break;  // Happens when penalty shootout completely ends.
            default: log<NUClear::WARN>("Unknown penalty shootout gamemode phase.");
        }
    }

    // ********************NORMAL GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::normal(const GameState& gameState,
                                const Phase& phase,
                                const FieldDescription& fieldDescription,
                                const Field& field,
                                const Ball& ball) {
        switch (phase.value) {
            // Beginning of game and half time
            case Phase::INITIAL: normalInitial(); break;
            // After initial, robots position on their half of the field.
            case Phase::READY: normalReady(gameState, fieldDescription); break;
            // Happens after ready. Robot should stop moving.
            case Phase::SET: normalSet(); break;
            // After set, main game where we should walk to ball and kick.
            case Phase::PLAYING: normalPlaying(field, ball, fieldDescription); break;
            case Phase::FINISHED: normalFinished(); break;  // Game has finished.
            case Phase::TIMEOUT: normalTimeout(); break;    // A pause in playing. Not in simulation.
            default: log<NUClear::WARN>("Unknown normal gamemode phase.");
        }
    }

    // ********************PENALTY GAMEMODE STATES********************************
    void SoccerStrategy::penaltyShootoutInitial() {
        // There's no point in doing anything since we'll be teleported over to a penalty shootout position
        standStill();
        currentState = Behaviour::State::INITIAL;
    }

    void SoccerStrategy::penaltyShootoutReady() {
        // Should not happen
        currentState = Behaviour::State::READY;
    }

    void SoccerStrategy::penaltyShootoutSet(const FieldDescription& fieldDescription) {
        emit(std::make_unique<ResetWebotsServos>());         // we were teleported, so reset
        hasKicked = false;                                   // reset the hasKicked flag between kicks
        penaltyShootoutLocalisationReset(fieldDescription);  // Reset localisation
        standStill();
        currentState = Behaviour::State::SET;
    }

    void SoccerStrategy::penaltyShootoutPlaying(const Field& field, const Ball& ball) {
        // Execute penalty kick script once if we haven't yet, and if we are not goalie
        if (!hasKicked && team_kicking_off == GameEvents::Context::TEAM) {
            emit(std::make_unique<KickScriptCommand>(LimbID::RIGHT_LEG, KickCommandType::PENALTY));
            hasKicked    = true;  // Set this so we do not kick again, otherwise the script will keep trying to execute
            currentState = Behaviour::State::SHOOTOUT;
        }
        // If we are not kicking off then be a goalie
        else if (team_kicking_off == GameEvents::Context::OPPONENT) {
            find({FieldTarget(FieldTarget::Target::BALL)});
            goalieWalk(field, ball);
            currentState = Behaviour::State::GOALIE_WALK;
        }
        // Else we are not a goalie but we've already kicked. Current behaviour is to do nothing.
        else {
            standStill();
            currentState = Behaviour::State::SHOOTOUT;
        }
    }

    void SoccerStrategy::penaltyShootoutTimeout() {
        // Don't need to do anything
        standStill();
        currentState = Behaviour::State::TIMEOUT;
    }

    void SoccerStrategy::penaltyShootoutFinished() {
        // Don't need to do anything
        standStill();
        currentState = Behaviour::State::FINISHED;
    }

    // ********************NORMAL GAMEMODE STATES********************************
    void SoccerStrategy::normalInitial() {
        standStill();
        find({FieldTarget(FieldTarget::Target::SELF)});

        if (resetInInitial) {
            initialLocalisationReset();
            emit(std::make_unique<ResetWebotsServos>());
            resetInInitial = false;
        }

        currentState = Behaviour::State::INITIAL;
    }

    void SoccerStrategy::normalReady(const GameState& gameState, const FieldDescription& fieldDescription) {
        if (gameState.data.our_kick_off) {
            walkTo(fieldDescription, cfg_.start_position_offensive);
        }
        else {
            walkTo(fieldDescription, cfg_.start_position_defensive);
        }
        find({FieldTarget(FieldTarget::Target::SELF)});
        currentState = Behaviour::State::READY;
    }

    void SoccerStrategy::normalSet() {
        standStill();
        find({FieldTarget(FieldTarget::Target::BALL)});
        resetInInitial = true;
        currentState   = Behaviour::State::SET;
    }

    void SoccerStrategy::normalPlaying(const Field& field, const Ball& ball, const FieldDescription& fieldDescription) {
        if (penalised() && !cfg_.forcePlaying) {  // penalised
            standStill();
            find({FieldTarget(FieldTarget::Target::SELF)});
            currentState = Behaviour::State::PENALISED;
        }
        else if (cfg_.is_goalie) {  // goalie
            find({FieldTarget(FieldTarget::Target::BALL)});
            goalieWalk(field, ball);
            currentState = Behaviour::State::GOALIE_WALK;
        }
        else {
            if (NUClear::clock::now() - ballLastMeasured
                < cfg_.ball_last_seen_max_time) {  // ball has been seen recently
                find({FieldTarget(FieldTarget::Target::BALL)});
                walkTo(fieldDescription, FieldTarget::Target::BALL);
                currentState = Behaviour::State::WALK_TO_BALL;
            }
            else {  // ball has not been seen recently
                Eigen::Affine2d position(field.position);
                if (position.translation().norm() > 1) {  // a long way away from centre
                    // walk to centre of field
                    find({FieldTarget(FieldTarget::Target::BALL)});
                    walkTo(fieldDescription, Eigen::Vector2d::Zero());
                    currentState = Behaviour::State::MOVE_TO_CENTRE;
                }
                else {
                    find({FieldTarget(FieldTarget::Target::BALL)});
                    walkTo(fieldDescription, FieldTarget::Target::BALL);
                    currentState = Behaviour::State::SEARCH_FOR_BALL;
                }
            }
        }
    }

    void SoccerStrategy::normalFinished() {
        standStill();
        find({FieldTarget(FieldTarget::Target::SELF)});
        currentState = Behaviour::State::FINISHED;
    }

    void SoccerStrategy::normalTimeout() {
        standStill();
        find({FieldTarget(FieldTarget::Target::SELF)});
        currentState = Behaviour::State::TIMEOUT;
    }

    void SoccerStrategy::initialLocalisationReset() {
        emit(std::make_unique<ResetRobotHypotheses>());
        auto ball_reset        = std::make_unique<ResetBallHypotheses>();
        ball_reset->self_reset = true;
        emit(ball_reset);
    }

    // **************************** LOCALISATION RESETS ****************************
    void SoccerStrategy::penaltyShootoutLocalisationReset(const FieldDescription& fieldDescription) {
        auto robot_reset = std::make_unique<ResetRobotHypotheses>();

        ResetRobotHypotheses::Self selfSideBaseLine;
        selfSideBaseLine.rTFf = Eigen::Vector2d(
            (-fieldDescription.dimensions.field_length / 2.0) + fieldDescription.dimensions.penalty_mark_distance,
            0.0);
        selfSideBaseLine.covariance  = Eigen::Vector2d::Constant(0.01).asDiagonal();
        selfSideBaseLine.heading     = -M_PI;
        selfSideBaseLine.heading_var = 0.005;

        robot_reset->hypotheses.push_back(selfSideBaseLine);

        emit(robot_reset);

        auto ball_reset = std::make_unique<ResetBallHypotheses>();

        ResetBallHypotheses::Ball atFeet;
        atFeet.rBWw       = Eigen::Vector2d(0.2, 0);
        atFeet.covariance = Eigen::Vector2d::Constant(0.01).asDiagonal();

        ball_reset->hypotheses.push_back(atFeet);
        ball_reset->self_reset = true;

        emit(ball_reset);
    }

    void SoccerStrategy::unpenalisedLocalisationReset() {
        emit(std::make_unique<ResetRobotHypotheses>());

        // TODO(BehaviourTeam): This should do some random distribution or something as we don't know where the ball is
        auto ball_reset        = std::make_unique<ResetBallHypotheses>();
        ball_reset->self_reset = true;
        emit(ball_reset);
    }

    // ******************* MOTIONS *************************************************
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

    bool SoccerStrategy::pickedUp(const Sensors& sensors) const {
        bool feetOffGround = !sensors.feet[BodySide::LEFT].down && !sensors.feet[BodySide::RIGHT].down;
        return false && feetOffGround && !isGettingUp && sensors.Htw(2, 2) < 0.92 && sensors.Htw(2, 2) > 0.88;
    }

    bool SoccerStrategy::penalised() const {
        return selfPenalised;
    }

    bool SoccerStrategy::ballDistance(const Ball& ball) {
        return ball.position.norm() != 0.0;
    }

    void SoccerStrategy::find(const std::vector<FieldTarget>& fieldObjects) {
        // Create the soccer object priority pointer and initialise each value to 0.
        auto soccerObjectPriority  = std::make_unique<SoccerObjectPriority>();
        soccerObjectPriority->ball = 0;
        soccerObjectPriority->goal = 0;
        soccerObjectPriority->line = 0;
        for (const auto& fieldObject : fieldObjects) {
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

    Eigen::Vector2d SoccerStrategy::getKickPlan(const Field& field, const FieldDescription& fieldDescription) {
        // Defines the box within in which the kick target is changed from the centre
        // of the oppposition goal to the perpendicular distance from the robot to the goal

        const float maxKickRange =
            0.6;  // TODO(BehaviourTeam): make configurable, only want to change at the last kick to avoid smart goalies
        const float xTakeOverBox = maxKickRange;
        const float error        = 0.05;
        const float buffer       = error + 2.0f * fieldDescription.ball_radius;          // 15cm
        const float yTakeOverBox = fieldDescription.dimensions.goal_width / 2 - buffer;  // 90-15 = 75cm
        Eigen::Affine2d position(field.position);
        const float xRobot = position.translation().x();
        const float yRobot = position.translation().y();
        Eigen::Vector2d newTarget{};

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
        auto motionCommand = std::make_unique<MotionCommand>();

        float timeSinceBallSeen =
            std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - ballLastMeasured).count();

        if (timeSinceBallSeen < cfg_.goalie_command_timeout) {

            Eigen::Affine2d position(field.position);
            const float fieldBearing  = Eigen::Rotation2Dd(position.rotation()).angle();
            const int signBearing     = fieldBearing > 0 ? 1 : -1;
            const float rotationSpeed = -signBearing
                                        * std::fmin(std::fabs(cfg_.goalie_rotation_speed_factor * fieldBearing),
                                                    cfg_.goalie_max_rotation_speed);

            const int signTranslation = ball.position.y() > 0 ? 1 : -1;
            const float translationSpeed =
                signTranslation
                * std::fmin(std::fabs(cfg_.goalie_translation_speed_factor * ball.position[1]),
                            cfg_.goalie_max_translation_speed);

            Eigen::Affine2d cmd{};
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
