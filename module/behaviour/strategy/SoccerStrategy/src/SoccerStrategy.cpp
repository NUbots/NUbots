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
#include <cmath>

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
    using LimbID      = utility::input::LimbID;

    using utility::input::LimbID;
    using utility::math::coordinates::reciprocalSphericalToCartesian;
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

            cfg.walk_to_ready_time = config["walk_to_ready_time"].as<int>();

            cfg.kicking_distance_threshold = config["kicking_distance_threshold"].as<float>();

            cfg.kicking_angle_threshold = config["kicking_angle_threshold"].as<float>();

            cfg.rBTt_smoothing_factor = config["rBTt_smoothing_factor"].as<float>();
        });

        on<Trigger<VisionBalls>, With<Sensors>>().then([this](const VisionBalls& balls, const Sensors& sensors) {
            if (balls.balls.size() > 0) {
                ball_last_measured = NUClear::clock::now();
                // Get the latest vision ball measurement in camera space

                Eigen::Vector3f rBCc =
                    reciprocalSphericalToCartesian(balls.balls[0].measurements[0].srBCc.cast<float>());

                Eigen::Affine3f Htc(sensors.Htw.cast<float>() * balls.Hcw.inverse().cast<float>());

                rBTt = Htc * rBCc;

                auto raw_lowest_distance = std::sqrt(std::pow(rBTt_smoothed.x(), 2) + std::pow(rBTt_smoothed.y(), 2));

                // TODO(BehaviourTeam Thailand): This is due to be moved into the VisionBall module

                for (const auto& ball : balls.balls) {
                    rBCc = reciprocalSphericalToCartesian(ball.measurements[0].srBCc.cast<float>());
                    Eigen::Affine3f Htc(sensors.Htw.cast<float>() * balls.Hcw.inverse().cast<float>());
                    auto temp_rBTt = Htc * rBCc;

                    if (std::sqrt(std::pow(rBTt_smoothed.x(), 2) + std::pow(rBTt_smoothed.y(), 2))
                        < raw_lowest_distance) {
                        raw_lowest_distance =
                            std::sqrt(std::pow(rBTt_smoothed.x(), 2) + std::pow(rBTt_smoothed.y(), 2));
                        rBTt = temp_rBTt;
                    }
                }

                rBTt = Htc * rBCc;

                // Apply exponential filter to rBTt
                rBTt_smoothed = cfg.rBTt_smoothing_factor * rBTt + (1.0 - cfg.rBTt_smoothing_factor) * rBTt_smoothed;

                distance_to_ball = std::sqrt(std::pow(rBTt.x(), 2) + std::pow(rBTt.y(), 2));
                angle_to_ball    = std::abs(std::atan2(rBTt.y(), rBTt.x()));
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
                        current_state = Behaviour::State::PICKED_UP;
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
                    if (current_state != previous_state) {
                        previous_state = current_state;
                        emit(std::make_unique<Behaviour::State>(current_state));
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
    void SoccerStrategy::normal(const message::input::GameState& game_state, const Phase& phase) {
        switch (phase.value) {
            // Beginning of game and half time
            case Phase::INITIAL: normal_initial(); break;
            // After initial, robots position on their half of the field.
            case Phase::READY: normal_ready(game_state); break;
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
        current_state = Behaviour::State::INITIAL;
    }

    void SoccerStrategy::penalty_shootout_ready() {
        // Should not happen
        current_state = Behaviour::State::READY;
    }

    void SoccerStrategy::penalty_shootout_set(const FieldDescription& field_description) {
        stand_still();
        current_state = Behaviour::State::SET;
    }

    void SoccerStrategy::penalty_shootout_playing(const Field& field, const Ball& ball) {
        // Execute penalty kick script once if we haven't yet, and if we are not goalie
        if (team_kicking_off == GameEvents::Context::TEAM) {
            if (NUClear::clock::now() - ball_last_measured < cfg.ball_last_seen_max_time) {
                // Ball has been seen recently
                play();
            }
            else {
                // Ball has not been seen recently, request walk planner to rotate on the spot
                stand_still();
            }
            current_state = Behaviour::State::SHOOTOUT;
        }
        // If we are not kicking off then be a goalie
        else if (team_kicking_off == GameEvents::Context::OPPONENT) {
            goalie_walk(field, ball);
            current_state = Behaviour::State::GOALIE_WALK;
        }
    }

    void SoccerStrategy::penalty_shootout_timeout() {
        // Don't need to do anything
        stand_still();
        current_state = Behaviour::State::TIMEOUT;
    }

    void SoccerStrategy::penalty_shootout_finished() {
        // Don't need to do anything
        stand_still();
        current_state = Behaviour::State::FINISHED;
    }

    // ********************NORMAL GAMEMODE STATES********************************
    void SoccerStrategy::normal_initial() {
        stand_still();
        if (reset_in_initial) {
            initial_localisation_reset();
            emit(std::make_unique<ResetWebotsServos>());
            reset_in_initial = false;
        }

        current_state = Behaviour::State::INITIAL;
    }

    void SoccerStrategy::normal_ready(const GameState& game_state) {
        if (penalised()) {  // penalised
            stand_still();
            current_state = Behaviour::State::PENALISED;
        }
        else {
            if (!game_state.data.first_half & !is_reset_half) {
                is_reset_half            = true;
                started_walking_to_ready = false;
            }
            if (!started_walking_to_ready) {
                started_walking_to_ready_at = NUClear::clock::now();
                started_walking_to_ready    = true;
            }
            // Walk forwards for cfg.walk_to_ready_time seconds
            if (NUClear::clock::now() - started_walking_to_ready_at
                < std::chrono::milliseconds(cfg.walk_to_ready_time * 1000)) {
                emit(std::make_unique<MotionCommand>(utility::behaviour::WalkToReady()));
            }
            else {
                stand_still();
            }
            current_state = Behaviour::State::READY;
        }
    }

    void SoccerStrategy::normal_set() {
        stand_still();
        reset_in_initial = true;
        current_state    = Behaviour::State::SET;
    }

    void SoccerStrategy::normal_playing() {
        if (penalised() && !cfg.force_playing) {
            // We are penalised, stand still
            stand_still();
            current_state = Behaviour::State::PENALISED;
        }
        else {

            log<NUClear::DEBUG>("Distance to ball :",
                                distance_to_ball,
                                " with expected kicking distance: ",
                                cfg.kicking_distance_threshold);

            log<NUClear::DEBUG>("Angle to ball",
                                angle_to_ball,
                                " with expected kicking angle: ",
                                cfg.kicking_angle_threshold);

            if (NUClear::clock::now() - ball_last_measured < cfg.ball_last_seen_max_time) {
                // Ball has been seen recently
                play();
                current_state = Behaviour::State::WALK_TO_BALL;
            }
            else {
                // Ball has not been seen recently, request walk planner to rotate on the spot
                find();
                current_state = Behaviour::State::SEARCH_FOR_BALL;
            }
        }
    }

    void SoccerStrategy::normal_finished() {
        stand_still();
        current_state = Behaviour::State::FINISHED;
    }

    void SoccerStrategy::normal_timeout() {
        stand_still();
        current_state = Behaviour::State::TIMEOUT;
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

        // TODO(BehaviourTeam): This should do some random distribution or something as we don't know where the ball
        // is
        auto ball_reset        = std::make_unique<ResetBallHypotheses>();
        ball_reset->self_reset = true;
        emit(ball_reset);
    }

    // ******************* MOTIONS *************************************************
    void SoccerStrategy::stand_still() {
        emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
    }

    bool SoccerStrategy::picked_up(const Sensors& sensors) const {
        bool feet_off_ground = !sensors.feet[BodySide::LEFT].down && !sensors.feet[BodySide::RIGHT].down;
        return false && feet_off_ground && !is_getting_up && sensors.Htw(2, 2) < 0.92 && sensors.Htw(2, 2) > 0.88;
    }

    bool SoccerStrategy::penalised() const {
        return self_penalised;
    }

    void SoccerStrategy::find() {
        emit(std::make_unique<MotionCommand>(utility::behaviour::RotateOnSpot()));
    }

    void SoccerStrategy::play() {
        if (distance_to_ball < cfg.kicking_distance_threshold && angle_to_ball < cfg.kicking_angle_threshold) {
            // Ball is close enough and in the correct direction to kick
            if (rBTt_smoothed.y() > 0) {
                log<NUClear::DEBUG>("We are close to the ball, kick it left");
                emit(std::make_unique<KickScriptCommand>(LimbID::LEFT_LEG, KickCommandType::NORMAL));
            }
            else {
                log<NUClear::DEBUG>("We are close to the ball, kick it right");
                emit(std::make_unique<KickScriptCommand>(LimbID::RIGHT_LEG, KickCommandType::NORMAL));
            }
        }
        else {
            // Request walk planner to walk to the ball
            emit(std::make_unique<MotionCommand>(utility::behaviour::BallApproach()));
        }
    }

    Eigen::Vector2d SoccerStrategy::get_kick_plan(const Field& field, const FieldDescription& field_description) {
        // Defines the box within in which the kick target is changed from the centre
        // of the oppposition goal to the perpendicular distance from the robot to the goal

        const float max_kick_range = 0.6;  // TODO(BehaviourTeam): make configurable, only want to change at the
                                           // last kick to avoid smart goalies
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
