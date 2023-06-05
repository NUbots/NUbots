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

#include "message/actuation/BodySide.hpp"
#include "message/behaviour/MotionCommand.hpp"
#include "message/behaviour/Nod.hpp"
#include "message/input/GameEvents.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/localisation/ResetBallHypotheses.hpp"
#include "message/localisation/ResetRobotHypotheses.hpp"
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
    using message::behaviour::MotionCommand;
    using message::behaviour::Nod;
    using message::input::GameEvents;
    using message::input::GameState;
    using Phase          = message::input::GameState::Data::Phase;
    using KickOffTeam    = message::input::GameEvents::KickOffTeam;
    using Penalisation   = message::input::GameEvents::Penalisation;
    using Unpenalisation = message::input::GameEvents::Unpenalisation;
    using GameMode       = message::input::GameState::Data::Mode;
    using message::actuation::BodySide;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::ResetBallHypotheses;
    using message::localisation::ResetRobotHypotheses;
    using message::motion::ExecuteGetup;
    using message::motion::KickCommandType;
    using message::motion::KickScriptCommand;
    using message::motion::KillGetup;
    using message::platform::ButtonMiddleDown;
    using message::platform::ResetWebotsServos;
    using message::support::FieldDescription;

    using FilteredBall = message::localisation::FilteredBall;

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

            cfg.localisation_interval = std::chrono::duration_cast<NUClear::clock::duration>(
                duration<double>(config["localisation_interval"].as<double>()));
            cfg.localisation_duration = std::chrono::duration_cast<NUClear::clock::duration>(
                duration<double>(config["localisation_duration"].as<double>()));

            cfg.start_position_offensive = config["start_position_offensive"].as<Expression>();
            cfg.start_position_defensive = config["start_position_defensive"].as<Expression>();

            cfg.is_goalie                = config["is_goalie"].as<bool>();
            cfg.goalie_max_ball_distance = config["goalie_max_ball_distance"].as<float>();

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
           Optional<With<FilteredBall>>,
           Single>()
            .then([this](const Sensors& sensors,
                         const GameState& game_state,
                         const Phase& phase,
                         const std::shared_ptr<const FilteredBall>& ball) {
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
                            normal_playing(ball);
                        }
                        // Overide SoccerStrategy and force penalty mode in playing phase
                        else if (cfg.force_penalty_shootout) {
                            team_kicking_off = GameEvents::Context::TEAM;
                            penalty_shootout_playing(ball);
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
                                case GameMode::DIRECT_FREEKICK: direct_freekick(game_state); break;
                                case GameMode::INDIRECT_FREEKICK: indirect_freekick(game_state); break;
                                case GameMode::PENALTYKICK: penalty_kick(game_state); break;
                                case GameMode::CORNER_KICK: corner_kick(game_state); break;
                                case GameMode::GOAL_KICK: goal_kick(game_state); break;
                                case GameMode::THROW_IN: throw_in(game_state); break;
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
    void SoccerStrategy::penalty_shootout(const Phase& phase, const std::shared_ptr<const FilteredBall>& ball) {
        switch (phase.value) {
            case Phase::INITIAL: penalty_shootout_initial(); break;      // Happens at beginning.
            case Phase::READY: penalty_shootout_ready(); break;          // Should not happen.
            case Phase::SET: penalty_shootout_set(); break;              // Happens on beginning of each kick try.
            case Phase::PLAYING: penalty_shootout_playing(ball); break;  // Either kicking or goalie.
            case Phase::TIMEOUT: penalty_shootout_timeout(); break;      // A pause in playing. Not in simulation.
            case Phase::FINISHED: penalty_shootout_finished(); break;  // Happens when penalty shootout completely ends.
            default: log<NUClear::WARN>("Unknown penalty shootout gamemode phase.");
        }
    }

    // ********************NORMAL GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::normal(const message::input::GameState& game_state,
                                const Phase& phase,
                                const std::shared_ptr<const FilteredBall>& ball) {
        switch (phase.value) {
            // Beginning of game and half time
            case Phase::INITIAL: normal_initial(); break;
            // After initial, robots position on their half of the field.
            case Phase::READY: normal_ready(game_state); break;
            // Happens after ready. Robot should stop moving.
            case Phase::SET: normal_set(); break;
            // After set, main game where we should walk to ball and kick.
            case Phase::PLAYING: normal_playing(ball); break;
            case Phase::FINISHED: normal_finished(); break;  // Game has finished.
            case Phase::TIMEOUT: normal_timeout(); break;    // A pause in playing. Not in simulation.
            default: log<NUClear::WARN>("Unknown normal gamemode phase.");
        }
    }

    // ********************DIRECT_FREEKICK GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::direct_freekick(const message::input::GameState& game_state) {
        if (game_state.data.secondary_state.team_performing != game_state.data.team.team_id) {
            direct_freekick_wait();
        }
        else {
            switch (game_state.data.secondary_state.sub_mode) {
                // Beginning of game and half time
                case 0: direct_freekick_wait(); break;
                // After initial, robots position on their half of the field.
                case 1: direct_freekick_placing(); break;
                // Happens after ready. Robot should stop moving.
                case 2: direct_freekick_end_placing(); break;
                // After set, main game where we should walk to ball and kick.
                default: log<NUClear::WARN>("Unknown sub mode.");
            }
        }
    }

    // ********************INDIRECT_FREEKICK GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::indirect_freekick(const message::input::GameState& game_state) {
        direct_freekick(game_state);
    }

    // ********************PENTALTY_KICK GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::penalty_kick(const message::input::GameState& game_state) {
        direct_freekick(game_state);
    }

    // ********************CORNER GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::corner_kick(const message::input::GameState& game_state) {
        direct_freekick(game_state);
    }

    // ********************GOAL GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::goal_kick(const message::input::GameState& game_state) {
        direct_freekick(game_state);
    }

    // ********************THROW IN GAMEMODE STATE MACHINE********************************
    void SoccerStrategy::throw_in(const message::input::GameState& game_state) {
        direct_freekick(game_state);
    }

    // ********************PENALTY GAMEMODE STATES********************************
    void SoccerStrategy::penalty_shootout_initial() {
        // There's no point in doing anything since we'll be teleported over to a penalty shootout position
        stand_still();
        current_state = Behaviour::State::INITIAL;
    }

    void SoccerStrategy::penalty_shootout_ready() {
        // Should be unreachable
        log<NUClear::ERROR>("penalty_shootout_ready should be unreachable!");
        current_state = Behaviour::State::READY;
    }

    void SoccerStrategy::penalty_shootout_set() {
        stand_still();
        current_state = Behaviour::State::SET;
    }

    void SoccerStrategy::penalty_shootout_playing(const std::shared_ptr<const FilteredBall>& ball) {
        // Execute penalty kick script once if we haven't yet, and if we are not goalie (team_kicking_off will not be us
        // if we are the goalie)
        log<NUClear::DEBUG>("Entering penalty shootout playing...");
        if (team_kicking_off == GameEvents::Context::TEAM) {
            // Go kick the ball directly into the goals... i.e. regular playing
            play(ball);
            current_state = Behaviour::State::SHOOTOUT;
        }
        else {
            // TODO(BehaviourTeam): Can add a dive or walk in-front of the ball here to attempt to block in future
            stand_still();
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

    void SoccerStrategy::normal_playing(const std::shared_ptr<const FilteredBall>& ball) {
        if (penalised() && !cfg.force_playing) {
            // We are penalised, stand still
            stand_still();
            current_state = Behaviour::State::PENALISED;
        }
        else {
            if (ball && cfg.is_goalie && ball->rBTt.norm() > cfg.goalie_max_ball_distance) {
                // We are goalie and the ball is too far away, stand still
                stand_still();
                current_state = Behaviour::State::SEARCH_FOR_BALL;
                // current_state = Behaviour::State::GOALIE_WALK;
            }
            else if (ball && NUClear::clock::now() - ball->time_of_measurement < cfg.ball_last_seen_max_time) {
                // We are not goalie or the ball is close enough, request walk planner to walk to the ball if
                // ball has been seen recently

                play(ball);
                current_state = Behaviour::State::WALK_TO_BALL;
            }
            else {
                // Ball has not been seen recently, request walk planner to rotate on the spot
                if (!cfg.is_goalie) {
                    find(ball);
                }
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

    // ********************DIRECT FREEKICK GAMEMODE STATES********************************
    void SoccerStrategy::direct_freekick_wait() {
        stand_still();
    }

    void SoccerStrategy::direct_freekick_placing() {
        if (NUClear::clock::now() - ball_last_measured < cfg.ball_last_seen_max_time) {
            // Ball has been seen recently, request walk planner to walk to the ball
            play();
            currentState = Behaviour::State::WALK_TO_BALL;
        }
        else {
            // Ball has not been seen recently, request walk planner to rotate on the spot
            find();
            currentState = Behaviour::State::SEARCH_FOR_BALL;
        }
    }

    void SoccerStrategy::direct_freekick_end_placing() {
        emit(std::make_unique<KickScriptCommand>(LimbID::RIGHT_LEG, KickCommandType::NORMAL));
    }


    // **************************** LOCALISATION RESETS ****************************
    void SoccerStrategy::penalty_shootout_localisation_reset() {}

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

    bool SoccerStrategy::picked_up(const Sensors& sensors) const {
        bool feet_off_ground = !sensors.feet[BodySide::LEFT].down && !sensors.feet[BodySide::RIGHT].down;
        return false && feet_off_ground && !is_getting_up && sensors.Htw(2, 2) < 0.92 && sensors.Htw(2, 2) > 0.88;
    }

    bool SoccerStrategy::penalised() const {
        return self_penalised;
    }

    void SoccerStrategy::find(const std::shared_ptr<const FilteredBall>& ball) {
        if (ball && ball->rBTt.y() < 0.0) {
            emit(std::make_unique<MotionCommand>(utility::behaviour::RotateOnSpot(true)));
        }
        else {
            emit(std::make_unique<MotionCommand>(utility::behaviour::RotateOnSpot(false)));
        }
    }

    void SoccerStrategy::play(const std::shared_ptr<const FilteredBall>& ball) {
        float absolute_yaw_angle = std::abs(std::atan2(ball->rBTt.y(), ball->rBTt.x()));
        float distance_to_ball   = ball->rBTt.head(2).norm();
        if (ball && distance_to_ball < cfg.kicking_distance_threshold
            && absolute_yaw_angle < cfg.kicking_angle_threshold) {
            // We are in range, lets kick
            emit(std::make_unique<KickScriptCommand>(LimbID::RIGHT_LEG, KickCommandType::NORMAL));
        }
        else {
            // Request walk planner to walk to the ball
            emit(std::make_unique<MotionCommand>(utility::behaviour::BallApproach()));
        }

        if (log_level <= NUClear::DEBUG) {
            log<NUClear::DEBUG>("Distance to ball: ", distance_to_ball);
        }
    }

}  // namespace module::behaviour::strategy
