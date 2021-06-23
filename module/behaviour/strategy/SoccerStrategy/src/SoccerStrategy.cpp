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
#include "extension/Script.hpp"

#include "message/behaviour/MotionCommand.hpp"
#include "message/behaviour/Nod.hpp"
#include "message/behaviour/SoccerObjectPriority.hpp"
#include "message/input/GameEvents.hpp"
#include "message/input/Sensors.hpp"
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
    using extension::ExecuteScriptByName;

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
    using message::localisation::ResetRobotHypotheses;
    using message::motion::BodySide;
    using message::motion::ExecuteGetup;
    using message::motion::KickCommandType;
    using message::motion::KickScriptCommand;
    using message::motion::KillGetup;
    using message::platform::ButtonLeftDown;
    using message::platform::ButtonMiddleDown;
    using message::platform::ResetRawSensors;
    using message::support::FieldDescription;
    using VisionBalls = message::vision::Balls;
    using VisionGoals = message::vision::Goals;

    using utility::input::LimbID;
    using utility::support::Expression;

    using utility::support::Expression;

    SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , cfg_()
        , id(size_t(this) * size_t(this) - size_t(this))
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

        // // TODO: remove this horrible code
        // // Check to see if we are currently in the process of getting up.
        on<Trigger<ExecuteGetup>>().then([this] { isGettingUp = true; });

        // // Check to see if we have finished getting up.
        on<Trigger<KillGetup>>().then([this] { isGettingUp = false; });

        on<Trigger<Penalisation>>().then([this](const Penalisation& selfPenalisation) {
            emit(std::make_unique<ResetRawSensors>());
            if (selfPenalisation.context == GameEvents::Context::SELF) {
                selfPenalised = true;
            }
        });

        on<Trigger<Unpenalisation>, With<FieldDescription>>().then(
            [this](const Unpenalisation& selfPenalisation, const FieldDescription& fieldDescription) {
                if (selfPenalisation.context == GameEvents::Context::SELF) {
                    selfPenalised = false;

                    // TODO: isSideChecking = true;
                    // TODO: only do this once put down
                    unpenalisedLocalisationReset(fieldDescription);
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
                    // Switch gamemode statemachine based on GameController state
                    switch (mode) {
                        case GameMode::PENALTY_SHOOTOUT:
                            penaltyShootout(gameState, phase, fieldDescription, field, ball);
                            break;
                        case GameMode::NORMAL: normal(gameState, phase, fieldDescription, field, ball, mode); break;
                        case GameMode::OVERTIME: normal(gameState, phase, fieldDescription, field, ball, mode); break;
                        default: log<NUClear::WARN>("Game mode unknown.");
                    }
                }
                catch (std::runtime_error& err) {
                    log(err.what());
                    log("Runtime exception.");
                }
            });
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

        // make a motionCommand to emit
        std::unique_ptr<MotionCommand> motionCommand;

        // time since the ball has been seen, in microseconds, current clock - time the ball was last measured
        float timeSinceBallSeen =
            std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now() - ballLastMeasured).count()
            * 1e6;  // SUSS

        // timeSinceBallSeen < 1 (as stated in the cfg file SoccerStrategy.yaml)
        if (timeSinceBallSeen < cfg_.goalie_command_timeout) {

            // position == Hfw
            Eigen::Affine2d position(field.position);

            float fieldBearing  = Eigen::Rotation2Dd(position.rotation()).angle();
            int signBearing     = fieldBearing > 0 ? 1 : -1;
            float rotationSpeed = -signBearing
                                  * std::fmin(std::fabs(cfg_.goalie_rotation_speed_factor * fieldBearing),
                                              cfg_.goalie_max_rotation_speed);

            int signTranslation = ball.position.y() > 0 ? 1 : -1;
            float tmp           = signTranslation
                        * std::fmin(std::fabs(cfg_.goalie_translation_speed_factor * ball.position[1]),
                                    cfg_.goalie_max_translation_speed);

            float translationSpeed = std::fabs(fieldBearing) < cfg_.goalie_side_walk_angle_threshold ? tmp : 0;

            Eigen::Affine2d cmd;
            cmd.linear() = Eigen::Rotation2Dd(rotationSpeed).matrix();

            // initalise the x and y movements to zero
            cmd.translation() = Eigen::Vector2d(0, translationSpeed);

            // motion command set to a direct command
            motionCommand = std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(cmd));

            // //2D rotation of the robot is less than the angle threshold of the goalie side walk
            // if (std::fabs(fieldBearing) < cfg_.goalie_side_walk_angle_threshold) {
            //     motionCommand->walk_command.y() = translationSpeed;
            // }
        }
        // looking for the ball?? Do nothing??
        else {
            motionCommand =
                std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(Eigen::Affine2d::Identity()));
        }
        emit(std::move(motionCommand));
    }

    // ********************PENALTY GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::penaltyShootout(const message::input::GameState& gameState,
                                         message::input::GameState::Data::Phase phase,
                                         const message::support::FieldDescription& fieldDescription,
                                         const message::localisation::Field& field,
                                         const message::localisation::Ball& ball) {
        switch (phase.value) {
            case Phase::INITIAL: penaltyShootoutInitial(); break;    // Happens at beginning
            case Phase::SET: penaltyShootoutSet(); break;            // Happens on beginning of each kick try
            case Phase::FINISHED: penaltyShootoutFinished(); break;  // Happens when penalty shootout all ends
            case Phase::TIMEOUT: penaltyShootoutTimeout(); break;
            case Phase::READY: penaltyShootoutReady(); break;      // Should not happen
            case Phase::PLAYING: penaltyShootoutPlaying(); break;  // Either kicking or goalie
            default: log<NUClear::WARN>("Unknown penalty shootout gamemode phase.");
        }
    }

    // ********************NORMAL GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::normal(const message::input::GameState& gameState,
                                message::input::GameState::Data::Phase phase,
                                const message::support::FieldDescription& fieldDescription,
                                const message::localisation::Field& field,
                                const message::localisation::Ball& ball,
                                const message::input::GameState::Data::Mode& mode) {
        switch (phase.value) {
            case Phase::INITIAL: normalInitial(fieldDescription); break;
            case Phase::READY: normalReady(gameState, fieldDescription); break;
            case Phase::SET: normalSet(); break;
            case Phase::PLAYING: normalPlaying(field, ball, fieldDescription, mode); break;
            case Phase::FINISHED: normalFinished(); break;
            case Phase::TIMEOUT: normalTimeout(); break;
            default: log<NUClear::WARN>("Unknown normal gamemode phase.");
        }
    }

    // ********************PENALTY GAMEMODE STATES********************************
    void SoccerStrategy::penaltyShootoutSet() {
        emit(std::make_unique<ResetRawSensors>());
        // Reset the has kicked flag between kicks
        hasKicked = false;
    }

    void SoccerStrategy::penaltyShootoutPlaying() {
        // Execute penalty kick script once if we haven't yet and are not goalie
        if (!hasKicked && team_kicking_off == GameEvents::Context::TEAM) {
            emit(std::make_unique<KickScriptCommand>(LimbID::LEFT_LEG, KickCommandType::PENALTY));
            hasKicked = true;
        }
    }

    // ********************NORMAL GAMEMODE STATES********************************
    void SoccerStrategy::normalReady(const message::input::GameState& gameState,
                                     const message::support::FieldDescription& fieldDescription) {
        if (gameState.data.our_kick_off) {
            walkTo(fieldDescription, cfg_.start_position_offensive);
        }
        else {
            walkTo(fieldDescription, cfg_.start_position_defensive);
        }
        find({FieldTarget(FieldTarget::Target::SELF)});
    }

    void SoccerStrategy::normalSet() {
        standStill();
        find({FieldTarget(FieldTarget::Target::BALL)});
    }

    void SoccerStrategy::normalPlaying(const Field& field,
                                       const Ball& ball,
                                       const FieldDescription& fieldDescription,
                                       const GameMode& mode) {
        if (penalised() && !cfg_.forcePlaying) {  // penalised
            standStill();
            find({FieldTarget(FieldTarget::Target::SELF)});
        }
        else if (cfg_.is_goalie) {  // goalie
            find({FieldTarget(FieldTarget::Target::BALL)});
            goalieWalk(field, ball);
        }
        else {
            if (NUClear::clock::now() - ballLastMeasured
                < cfg_.ball_last_seen_max_time) {  // ball has been seen recently
                find({FieldTarget(FieldTarget::Target::BALL)});
                walkTo(fieldDescription, FieldTarget::Target::BALL);
            }
            else {  // ball has not been seen recently
                Eigen::Affine2d position(field.position);
                if (position.translation().norm() > 1) {  // a long way away from centre
                    // walk to centre of field
                    find({FieldTarget(FieldTarget::Target::BALL)});
                    walkTo(fieldDescription, Eigen::Vector2d::Zero());
                }
                else {
                    find({FieldTarget(FieldTarget::Target::BALL)});
                    walkTo(fieldDescription, FieldTarget::Target::BALL);
                }
            }
        }
    }

    void SoccerStrategy::normalFinished() {
        standStill();
        find({FieldTarget(FieldTarget::Target::SELF)});
    }

    void SoccerStrategy::normalTimeout() {
        standStill();
        find({FieldTarget(FieldTarget::Target::SELF)});
    }

}  // namespace module::behaviour::strategy
