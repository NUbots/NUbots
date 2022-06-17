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
#include "utility/math/coordinates.hpp"
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
    using utility::math::coordinates::sphericalToCartesian;
    using utility::support::Expression;

    using utility::support::Expression;

    SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("SoccerStrategy.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            using namespace std::chrono;
            cfg.ball_last_seen_max_time = duration_cast<NUClear::clock::duration>(
                duration<double>(config["ball_last_seen_max_time"].as<double>()));
            cfg.goal_last_seen_max_time = duration_cast<NUClear::clock::duration>(
                duration<double>(config["goal_last_seen_max_time"].as<double>()));

            cfg.localisation_interval =
                duration_cast<NUClear::clock::duration>(duration<double>(config["localisation_interval"].as<double>()));
            cfg.localisation_duration =
                duration_cast<NUClear::clock::duration>(duration<double>(config["localisation_duration"].as<double>()));

            cfg.start_position_offensive = config["start_position_offensive"].as<Expression>();
            cfg.start_position_defensive = config["start_position_defensive"].as<Expression>();

            cfg.is_goalie = config["is_goalie"].as<bool>();

            // Use configuration here from file SoccerStrategy.yaml
            cfg.goalie_command_timeout           = config["goalie_command_timeout"].as<float>();
            cfg.goalie_rotation_speed_factor     = config["goalie_rotation_speed_factor"].as<float>();
            cfg.goalie_max_rotation_speed        = config["goalie_max_rotation_speed"].as<float>();
            cfg.goalie_translation_speed_factor  = config["goalie_translation_speed_factor"].as<float>();
            cfg.goalie_max_translation_speed     = config["goalie_max_translation_speed"].as<float>();
            cfg.goalie_side_walk_angle_threshold = config["goalie_side_walk_angle_threshold"].as<float>();

            cfg.force_playing          = config["force_playing"].as<bool>();
            cfg.force_penalty_shootout = config["force_penalty_shootout"].as<bool>();
        });

        on<Trigger<Field>, With<FieldDescription>>().then(
            [this](const Field& field, const FieldDescription& field_description) {
                Eigen::Vector2d kick_target = get_kick_plan(field, field_description);
                emit(std::make_unique<KickPlan>(KickPlan(kick_target, kick_type)));
            });

        // For checking last seen times
        on<Trigger<VisionBalls>>().then([this](const VisionBalls& balls) {
            if (!balls.balls.empty()) {
                ball_last_measured = NUClear::clock::now();
            }
        });

        on<Trigger<VisionGoals>>().then([this](const VisionGoals& goals) {
            if (!goals.goals.empty()) {
                goal_last_measured = NUClear::clock::now();
            }
        });

        // TODO(BehaviourTeam): remove this horrible code
        // Check to see if we are currently in the process of getting up.
        on<Trigger<ExecuteGetup>>().then([this] { is_getting_up = true; });

        // Check to see if we have finished getting up.
        on<Trigger<KillGetup>>().then([this] { is_getting_up = false; });

        on<Trigger<Penalisation>>().then([this](const Penalisation& self_penalisation) {
            if (self_penalisation.context == GameEvents::Context::SELF) {
                emit(std::make_unique<ResetWebotsServos>());
                self_penalised = true;
            }
        });

        on<Trigger<Unpenalisation>>().then([this](const Unpenalisation& self_penalisation) {
            if (self_penalisation.context == GameEvents::Context::SELF) {
                self_penalised = false;

                // TODO(BehaviourTeam): isSideChecking = true;
                // TODO(BehaviourTeam): only do this once put down
                unpenalised_localisation_reset();
            }
        });


        on<Trigger<ButtonMiddleDown>, Single>().then([this] {
            log("Middle button pressed!");
            if (!cfg.force_playing) {
                log("Force playing started.");
                emit(std::make_unique<Nod>(true));
                cfg.force_playing = true;
            }
        });

        on<Trigger<KickOffTeam>>().then(
            [this](const KickOffTeam& kick_off_team) { team_kicking_off = kick_off_team.context; });

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
                         const GameState& game_state,
                         const Phase& phase,
                         const FieldDescription& field_description,
                         const Field& field,
                         const Ball& ball) {
                try {
                    // If we're picked up, stand still
                    if (picked_up(sensors)) {
                        // TODO(BehaviourTeam): stand, no moving
                        stand_still();
                        currentState = Behaviour::State::PICKED_UP;
                    }
                    else {
                        // Overide SoccerStrategy and force normal mode in playing phase
                        if (cfg.force_playing) {
                            normal_playing();
                        }
                        // Overide SoccerStrategy and force penalty mode in playing phase
                        else if (cfg.force_penalty_shootout) {
                            team_kicking_off = GameEvents::Context::TEAM;
                            penalty_shootout_playing(field, ball);
                        }
                        else {
                            // Switch gamemode statemachine based on GameController mode
                            auto mode = game_state.data.mode.value;
                            switch (mode) {
                                case GameMode::PENALTY_SHOOTOUT:
                                    penalty_shootout(phase, field_description, field, ball);
                                    break;
                                case GameMode::NORMAL: normal(game_state, phase); break;
                                case GameMode::OVERTIME: normal(game_state, phase); break;
                                default: log<NUClear::WARN>("Game mode unknown.");
                            }
                        }
                    }

                    // Update current behaviour state if it has changed
                    if (currentState != previousState) {
                        previousState = currentState;
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
    void SoccerStrategy::penalty_shootout(const Phase& phase,
                                          const FieldDescription& field_description,
                                          const Field& field,
                                          const Ball& ball) {
        switch (phase.value) {
            case Phase::INITIAL: penalty_shootout_initial(); break;           // Happens at beginning.
            case Phase::READY: penalty_shootout_ready(); break;               // Should not happen.
            case Phase::SET: penalty_shootout_set(field_description); break;  // Happens on beginning of each kick try.
            case Phase::PLAYING: penalty_shootout_playing(field, ball); break;  // Either kicking or goalie.
            case Phase::TIMEOUT: penalty_shootout_timeout(); break;    // A pause in playing. Not in simulation.
            case Phase::FINISHED: penalty_shootout_finished(); break;  // Happens when penalty shootout completely ends.
            default: log<NUClear::WARN>("Unknown penalty shootout gamemode phase.");
        }
    }

    // ********************NORMAL GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::normal(const GameState& game_state, const Phase& phase) {
        switch (phase.value) {
            // Beginning of game and half time
            case Phase::INITIAL: normal_initial(); break;
            // After initial, robots position on their half of the field.
            case Phase::READY: normal_ready(); break;
            // Happens after ready. Robot should stop moving.
            case Phase::SET: normal_set(); break;
            // After set, main game where we should walk to ball and kick.
            case Phase::PLAYING: normal_playing(); break;
            case Phase::FINISHED: normal_finished(); break;  // Game has finished.
            case Phase::TIMEOUT: normal_timeout(); break;    // A pause in playing. Not in simulation.
            default: log<NUClear::WARN>("Unknown normal gamemode phase.");
        }
    }

    // ********************PENALTY GAMEMODE STATES********************************
    void SoccerStrategy::penalty_shootout_initial() {
        // There's no point in doing anything since we'll be teleported over to a penalty shootout position
        stand_still();
        currentState = Behaviour::State::INITIAL;
    }

    void SoccerStrategy::penalty_shootout_ready() {
        // Should not happen
        currentState = Behaviour::State::READY;
    }

    void SoccerStrategy::penalty_shootout_set(const FieldDescription& field_description) {
        emit(std::make_unique<ResetWebotsServos>());             // we were teleported, so reset
        has_kicked = false;                                      // reset the has_kicked flag between kicks
        penalty_shootout_localisation_reset(field_description);  // Reset localisation
        stand_still();
        currentState = Behaviour::State::SET;
    }

    void SoccerStrategy::penalty_shootout_playing(const Field& field, const Ball& ball) {
        // Execute penalty kick script once if we haven't yet, and if we are not goalie
        if (!has_kicked && team_kicking_off == GameEvents::Context::TEAM) {
            emit(std::make_unique<KickScriptCommand>(LimbID::RIGHT_LEG, KickCommandType::PENALTY));
            has_kicked   = true;  // Set this so we do not kick again, otherwise the script will keep trying to execute
            currentState = Behaviour::State::SHOOTOUT;
        }
        // If we are not kicking off then be a goalie
        else if (team_kicking_off == GameEvents::Context::OPPONENT) {
            find({FieldTarget(FieldTarget::Target::BALL)});
            goalie_walk(field, ball);
            currentState = Behaviour::State::GOALIE_WALK;
        }
        // Else we are not a goalie but we've already kicked. Current behaviour is to do nothing.
        else {
            stand_still();
            currentState = Behaviour::State::SHOOTOUT;
        }
    }

    void SoccerStrategy::penalty_shootout_timeout() {
        // Don't need to do anything
        stand_still();
        currentState = Behaviour::State::TIMEOUT;
    }

    void SoccerStrategy::penalty_shootout_finished() {
        // Don't need to do anything
        stand_still();
        currentState = Behaviour::State::FINISHED;
    }

    // ********************NORMAL GAMEMODE STATES********************************
    void SoccerStrategy::normal_initial() {
        stand_still();
        find({FieldTarget(FieldTarget::Target::SELF)});

        if (reset_in_initial) {
            initial_localisation_reset();
            emit(std::make_unique<ResetWebotsServos>());
            reset_in_initial = false;
        }

        currentState = Behaviour::State::INITIAL;
    }

    void SoccerStrategy::normal_ready() {
        stand_still();
        if (penalised()) {
            currentState = Behaviour::State::PENALISED;
        }
        else {
            currentState = Behaviour::State::READY;
        }
    }

    void SoccerStrategy::normal_set() {
        stand_still();
        find({FieldTarget(FieldTarget::Target::BALL)});
        reset_in_initial = true;
        currentState     = Behaviour::State::SET;
    }

    void SoccerStrategy::normal_playing() {
        if (penalised() && !cfg.force_playing) {  // penalised
            stand_still();
            currentState = Behaviour::State::PENALISED;
        }
        else {
            if (NUClear::clock::now() - ball_last_measured < cfg.ball_last_seen_max_time) {
                // Ball has been seen recently, request walk planner to walk to the ball
                Eigen::Vector2d kick_target(0, 0);
                emit(std::make_unique<MotionCommand>(utility::behaviour::BallApproach(kick_target)));
                currentState = Behaviour::State::WALK_TO_BALL;
            }
            else {
                // Ball has not been seen recently, request walk planner to rotate on the spot
                currentState = Behaviour::State::SEARCH_FOR_BALL;
                emit(std::make_unique<MotionCommand>(utility::behaviour::RotateOnSpot()));
            }
        }
    }

    void SoccerStrategy::normal_finished() {
        stand_still();
        find({FieldTarget(FieldTarget::Target::SELF)});
        currentState = Behaviour::State::FINISHED;
    }

    void SoccerStrategy::normal_timeout() {
        stand_still();
        find({FieldTarget(FieldTarget::Target::SELF)});
        currentState = Behaviour::State::TIMEOUT;
    }

    void SoccerStrategy::initial_localisation_reset() {
        emit(std::make_unique<ResetRobotHypotheses>());
        auto ball_reset        = std::make_unique<ResetBallHypotheses>();
        ball_reset->self_reset = true;
        emit(ball_reset);
    }

    // **************************** LOCALISATION RESETS ****************************
    void SoccerStrategy::penalty_shootout_localisation_reset(const FieldDescription& field_description) {
        auto robot_reset = std::make_unique<ResetRobotHypotheses>();

        ResetRobotHypotheses::Self self_side_base_line;
        self_side_base_line.rTFf = Eigen::Vector2d(
            (-field_description.dimensions.field_length / 2.0) + field_description.dimensions.penalty_mark_distance,
            0.0);
        self_side_base_line.covariance  = Eigen::Vector2d::Constant(0.01).asDiagonal();
        self_side_base_line.heading     = -M_PI;
        self_side_base_line.heading_var = 0.005;

        robot_reset->hypotheses.push_back(self_side_base_line);

        emit(robot_reset);

        auto ball_reset = std::make_unique<ResetBallHypotheses>();

        ResetBallHypotheses::Ball at_feet;
        at_feet.rBWw       = Eigen::Vector2d(0.2, 0);
        at_feet.covariance = Eigen::Vector2d::Constant(0.01).asDiagonal();

        ball_reset->hypotheses.push_back(at_feet);
        ball_reset->self_reset = true;

        emit(ball_reset);
    }

    void SoccerStrategy::unpenalised_localisation_reset() {
        emit(std::make_unique<ResetRobotHypotheses>());

        // TODO(BehaviourTeam): This should do some random distribution or something as we don't know where the ball is
        auto ball_reset        = std::make_unique<ResetBallHypotheses>();
        ball_reset->self_reset = true;
        emit(ball_reset);
    }

    // ******************* MOTIONS *************************************************
    void SoccerStrategy::stand_still() {
        emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
    }

    void SoccerStrategy::walk_to(const FieldDescription& field_description, const FieldTarget::Target& target) {
        if (target != FieldTarget::Target::BALL) {
            throw std::runtime_error("SoccerStrategy::walk_to: Only FieldTarget::Target::BALL is supported.");
        }

        Eigen::Vector2d enemy_goal(field_description.dimensions.field_length * 0.5, 0.0);

        emit(std::make_unique<MotionCommand>(utility::behaviour::BallApproach(enemy_goal)));
    }

    void SoccerStrategy::walk_to(const FieldDescription& field_description, const Eigen::Vector2d& position) {
        emit(std::make_unique<MotionCommand>(utility::behaviour::WalkToState(
            utility::math::transform::lookAt(position,
                                             Eigen::Vector2d(field_description.dimensions.field_length * 0.5, 0.0)))));
    }

    bool SoccerStrategy::picked_up(const Sensors& sensors) const {
        bool feet_off_ground = !sensors.feet[BodySide::LEFT].down && !sensors.feet[BodySide::RIGHT].down;
        return false && feet_off_ground && !is_getting_up && sensors.Htw(2, 2) < 0.92 && sensors.Htw(2, 2) > 0.88;
    }

    bool SoccerStrategy::penalised() const {
        return self_penalised;
    }

    bool SoccerStrategy::ball_distance(const Ball& ball) {
        return ball.position.norm() != 0.0;
    }

    void SoccerStrategy::find(const std::vector<FieldTarget>& field_objects) {
        // Create the soccer object priority pointer and initialise each value to 0.
        auto soccer_object_priority  = std::make_unique<SoccerObjectPriority>();
        soccer_object_priority->ball = 0;
        soccer_object_priority->goal = 0;
        soccer_object_priority->line = 0;
        for (const auto& field_object : field_objects) {
            switch (field_object.target.value) {
                case FieldTarget::Target::SELF: {
                    soccer_object_priority->goal        = 1;
                    soccer_object_priority->search_type = SearchType::GOAL_SEARCH;

                    break;
                }
                case FieldTarget::Target::BALL: {
                    soccer_object_priority->ball        = 1;
                    soccer_object_priority->search_type = SearchType::LOST;
                    break;
                }
                default: throw std::runtime_error("Soccer strategy attempted to find a bad object");
            }
        }
        emit(std::move(soccer_object_priority));
    }

    Eigen::Vector2d SoccerStrategy::get_kick_plan(const Field& field, const FieldDescription& field_description) {
        // Defines the box within in which the kick target is changed from the centre
        // of the oppposition goal to the perpendicular distance from the robot to the goal

        const float max_kick_range =
            0.6;  // TODO(BehaviourTeam): make configurable, only want to change at the last kick to avoid smart goalies
        const float x_take_over_box = max_kick_range;
        const float error           = 0.05;
        const float buffer          = error + 2.0f * field_description.ball_radius;          // 15cm
        const float y_take_over_box = field_description.dimensions.goal_width / 2 - buffer;  // 90-15 = 75cm
        Eigen::Affine2d position(field.position);
        const float x_robot = position.translation().x();
        const float y_robot = position.translation().y();
        Eigen::Vector2d new_target{};

        if ((field_description.dimensions.field_length * 0.5) - x_take_over_box < x_robot && -y_take_over_box < y_robot
            && y_robot < y_take_over_box) {
            // Aims for behind the point that gives the shortest distance
            new_target.x() =
                field_description.dimensions.field_length * 0.5 + field_description.dimensions.goal_depth * 0.5;
            new_target.y() = y_robot;
        }
        else {
            // Aims for the centre of the goal
            new_target.x() = field_description.dimensions.field_length * 0.5;
            new_target.y() = 0;
        }
        return new_target;
    }

    void SoccerStrategy::goalie_walk(const Field& field, const Ball& ball) {
        auto motion_command = std::make_unique<MotionCommand>();

        float time_since_ball_seen =
            std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - ball_last_measured)
                .count();

        if (time_since_ball_seen < cfg.goalie_command_timeout) {

            Eigen::Affine2d position(field.position);
            const float field_bearing = Eigen::Rotation2Dd(position.rotation()).angle();
            const int sign_bearing    = field_bearing > 0 ? 1 : -1;
            const float rotation_speed =
                -sign_bearing
                * std::fmin(std::fabs(cfg.goalie_rotation_speed_factor * field_bearing), cfg.goalie_max_rotation_speed);

            const int sign_translation = ball.position.y() > 0 ? 1 : -1;
            const float translation_speed =
                sign_translation
                * std::fmin(std::fabs(cfg.goalie_translation_speed_factor * ball.position[1]),
                            cfg.goalie_max_translation_speed);

            Eigen::Affine2d cmd{};
            cmd.linear()      = Eigen::Rotation2Dd(rotation_speed).matrix();
            cmd.translation() = Eigen::Vector2d::Zero();
            motion_command    = std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(cmd));
            if (std::fabs(field_bearing) < cfg.goalie_side_walk_angle_threshold) {
                motion_command->walk_command.y() = translation_speed;
            }
        }
        else {
            motion_command =
                std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(Eigen::Affine2d::Identity()));
        }
        emit(std::move(motion_command));
    }
}  // namespace module::behaviour::strategy
