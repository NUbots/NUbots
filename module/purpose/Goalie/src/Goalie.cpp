/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "Goalie.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/LookAround.hpp"
#include "message/planning/Save.hpp"
#include "message/purpose/Player.hpp"
#include "message/purpose/Purpose.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/math/euler.hpp"

namespace module::purpose {
    using extension::Configuration;

    using Phase      = message::input::GameState::Phase;
    using GoalieTask = message::purpose::Goalie;

    using message::input::GameState;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::planning::LookAround;
    using message::planning::Save;
    using message::purpose::Purpose;
    using message::purpose::SoccerPosition;
    using message::strategy::LookAtBall;
    using message::strategy::StandStill;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;

    using utility::math::euler::pos_rpy_to_transform;

    Goalie::Goalie(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Goalie.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Goalie.yaml
            this->log_level                = config["log_level"].as<NUClear::LogLevel>();
            cfg.waiting_distance_from_line = config["waiting_distance_from_line"].as<double>();
            cfg.goal_post_clearance        = config["goal_post_clearance"].as<double>();
            cfg.strafe_curve_depth         = config["strafe_curve_depth"].as<double>();
            cfg.localise_timeout           = std::chrono::seconds(config["localise_timeout"].as<int>());
            cfg.save_priority              = config["save_priority"].as<int>();
        });

        on<Provide<GoalieTask>,
           Optional<With<Ball>>,
           With<Field>,
           With<GameState>,
           With<GlobalConfig>,
           With<FieldDescription>,
           When<Phase, std::equal_to, Phase::PLAYING>>()
            .then([this](const std::shared_ptr<const Ball>& ball,
                         const Field& field,
                         const GameState& game_state,
                         const GlobalConfig& global_config,
                         const FieldDescription& fd) {
                // If play is stopped, stand still
                if (game_state.stopped) {
                    log<DEBUG>("Play is stopped, standing still.");
                    emit<Task>(std::make_unique<StandStill>());
                    return;
                }

                // Do not play until localisation has converged, e.g. when re-entering an already-playing
                // game after being unpenalised or restarted. Stand still and scan for field features so
                // the goalie doesn't walk to the wrong goal with a wrong or unconverged pose.
                if (!field.localised) {
                    // Start the timer the first time we notice we are not localised
                    if (!look_around_start) {
                        look_around_start = NUClear::clock::now();
                    }
                    // Only stand and look around for a limited time, then give up and play anyway
                    if (NUClear::clock::now() - *look_around_start < cfg.localise_timeout) {
                        log<DEBUG>("Not localised, standing still and looking around to localise.");
                        emit(std::make_unique<Purpose>(global_config.player_id,
                                                       SoccerPosition::UNKNOWN,
                                                       true,
                                                       false,
                                                       game_state.team.team_colour));
                        emit<Task>(std::make_unique<LookAround>(), 1);
                        emit<Task>(std::make_unique<StandStill>(), 1);
                        return;
                    }
                    log<DEBUG>("Look around timed out without localising, playing anyway.");
                }
                else {
                    // Localised, reset the timer for the next time localisation is lost
                    look_around_start.reset();
                }

                // General tasks
                emit<Task>(std::make_unique<LookAround>(), 1);  // Look around if can't see the ball
                emit<Task>(std::make_unique<LookAtBall>(), 2);  // Track the ball

                // If there's no ball message, we can't play, just look for the ball
                if (ball == nullptr) {
                    Eigen::Vector3d rPFf(fd.dimensions.field_length / 2.0 - cfg.waiting_distance_from_line,
                                         0.0,
                                         0.0);
                    Eigen::Isometry3d Hfr = pos_rpy_to_transform(rPFf, Eigen::Vector3d(0.0, 0.0, -M_PI));
                    emit<Task>(std::make_unique<WalkToFieldPosition>(Hfr, true));
                    log<DEBUG>("No ball message, waiting in middle of goals.");
                    emit(std::make_unique<Purpose>(global_config.player_id,
                                                   SoccerPosition::GOALIE,
                                                   true,
                                                   false,
                                                   game_state.team.team_colour));
                    return;
                }

                emit(std::make_unique<Purpose>(global_config.player_id,
                                               SoccerPosition::GOALIE,
                                               true,
                                               true,
                                               game_state.team.team_colour));

                // If the ball is in our half, play
                Eigen::Vector3d rBFf = field.Hfw * ball->rBWw;
                if (rBFf.x() < 0.0) {
                    // Not in our half, just stay in spot and keep looking
                    Eigen::Vector3d rPFf(fd.dimensions.field_length / 2.0 - cfg.waiting_distance_from_line,
                                         0.0,
                                         0.0);
                    Eigen::Isometry3d Hfr = pos_rpy_to_transform(rPFf, Eigen::Vector3d(0.0, 0.0, -M_PI));
                    emit<Task>(std::make_unique<WalkToFieldPosition>(Hfr, true));
                    log<DEBUG>("Ball not in our half, waiting in middle of goals.");

                    return;
                }

                // The ball is in our half, so defend the goal. The goalie never goes for the ball, so it never leaves
                // the penalty area: PlanSave positions it inside the area and takes over from the positioning walk
                // below when a shot is on its way, so it runs above it
                if (cfg.save_priority > 0) {
                    emit<Task>(std::make_unique<Save>(), cfg.save_priority);
                }

                // To position, we stay within the goals, following the ball's y position but bowing forward off the
                // line in a parabola as we approach the posts, and staying cfg.goal_post_clearance away from them
                double y_max = fd.dimensions.goal_width / 2.0 - cfg.goal_post_clearance;
                double y_position = std::clamp(rBFf.y(), -y_max, y_max);
                double x_offset   = cfg.strafe_curve_depth * (y_position / y_max) * (y_position / y_max);
                Eigen::Vector3d rPFf(fd.dimensions.field_length / 2.0 - x_offset, y_position, 0.0);
                Eigen::Isometry3d Hfr = pos_rpy_to_transform(rPFf, Eigen::Vector3d(0.0, 0.0, -M_PI));
                emit<Task>(std::make_unique<WalkToFieldPosition>(Hfr, true));
            });

        on<Provide<GoalieTask>,
           With<FieldDescription>,
           With<GameState>,
           With<GlobalConfig>,
           When<Phase, std::equal_to, Phase::READY>>()
            .then([this](const FieldDescription& fd, const GameState& game_state, const GlobalConfig& global_config) {
                // Walk to middle of goals
                Eigen::Vector3d rPFf(fd.dimensions.field_length / 2.0 - cfg.waiting_distance_from_line, 0.0, 0.0);
                Eigen::Isometry3d Hfr = pos_rpy_to_transform(rPFf, Eigen::Vector3d(0.0, 0.0, -M_PI));
                emit<Task>(std::make_unique<WalkToFieldPosition>(Hfr, true));

                // Send purpose
                emit(std::make_unique<Purpose>(global_config.player_id,
                                               SoccerPosition::GOALIE,
                                               true,
                                               true,
                                               game_state.team.team_colour));
            });

        // When not in playing or ready state, send off the team colour and unknown state
        on<Provide<GoalieTask>, With<GameState>, With<GlobalConfig>>().then(
            [this](const GameState& game_state, const GlobalConfig& global_config) {
                emit(std::make_unique<Purpose>(global_config.player_id,
                                               SoccerPosition::UNKNOWN,
                                               true,
                                               true,
                                               game_state.team.team_colour));
            });
    }

}  // namespace module::purpose
