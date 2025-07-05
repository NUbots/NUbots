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
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/LookAround.hpp"
#include "message/purpose/Player.hpp"
#include "message/purpose/Purpose.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/strategy/Who.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/math/euler.hpp"
#include "utility/strategy/soccer_strategy.hpp"

namespace module::purpose {
    using extension::Configuration;

    using Phase      = message::input::GameState::Phase;
    using GoalieTask = message::purpose::Goalie;

    using message::input::GameState;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::Robots;
    using message::planning::LookAround;
    using message::purpose::Attack;
    using message::purpose::FieldPlayer;
    using message::purpose::Purpose;
    using message::purpose::ReadyAttack;
    using message::purpose::SoccerPosition;
    using message::strategy::LookAtBall;
    using message::strategy::Search;
    using message::strategy::WalkToFieldPosition;
    using message::strategy::Who;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;

    using utility::math::euler::pos_rpy_to_transform;

    Goalie::Goalie(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Goalie.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Goalie.yaml
            this->log_level           = config["log_level"].as<NUClear::LogLevel>();
            cfg.equidistant_threshold = config["equidistant_threshold"].as<double>();
            cfg.ball_threshold        = config["ball_threshold"].as<double>();
        });

        on<Provide<GoalieTask>,
           Optional<With<Ball>>,
           Optional<With<Robots>>,
           With<Sensors>,
           With<Field>,
           With<GameState>,
           With<GlobalConfig>,
           With<FieldDescription>,
           When<Phase, std::equal_to, Phase::PLAYING>>()
            .then([this](const std::shared_ptr<const Ball>& ball,
                         const std::shared_ptr<const Robots>& robots,
                         const Sensors& sensors,
                         const Field& field,
                         const GameState& game_state,
                         const GlobalConfig& global_config,
                         const FieldDescription& fd) {
                // Determine if the game is in a penalty situation
                // Do this first to ensure the robot freezes if necessary
                bool penalty = game_state.mode.value >= GameState::Mode::DIRECT_FREEKICK
                               && game_state.mode.value <= GameState::Mode::THROW_IN;

                // If sub_mode is 0, the robot must freeze for referee ball repositioning
                // If sub_mode is 2, the robot must freeze until the referee calls execute
                if (penalty && (game_state.secondary_state.sub_mode == 0 || game_state.secondary_state.sub_mode == 2)) {
                    log<DEBUG>("We are in a freeze penalty situation, do nothing.");
                    return;
                }

                bool teammates_exist = std::find_if(robots->robots.begin(),
                                                    robots->robots.end(),
                                                    [](const auto& robot) { return robot.teammate; })
                                       != robots->robots.end();

                // Act like a field player
                if (!teammates_exist) {
                    log<DEBUG>("No teammates, act as field player.");
                    emit<Task>(std::make_unique<FieldPlayer>());
                    return;
                }

                // General tasks
                emit<Task>(std::make_unique<LookAround>(), 1);  // Look around if can't see the ball
                emit<Task>(std::make_unique<LookAtBall>(), 2);  // Track the ball

                // If there's no ball message, we can't play, just look for the ball
                if (ball == nullptr) {
                    Eigen::Vector3d rPFf(fd.dimensions.field_length / 2.0, 0.0, 0.0);
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

                // If the ball is in the defending third, play
                Eigen::Vector3d rBFf     = field.Hfw * ball->rBWw;
                double defending_third_x = fd.dimensions.field_length / 3.0;
                if (rBFf.x() < defending_third_x) {
                    // Not in defending third, just stay in spot and keep looking
                    Eigen::Vector3d rPFf(fd.dimensions.field_length / 2.0, 0.0, 0.0);
                    Eigen::Isometry3d Hfr = pos_rpy_to_transform(rPFf, Eigen::Vector3d(0.0, 0.0, -M_PI));
                    emit<Task>(std::make_unique<WalkToFieldPosition>(Hfr, true));
                    log<DEBUG>("Ball not in defending third, waiting in middle of goals.");

                    return;
                }

                // Make an ignore list with inactive teammates
                std::vector<unsigned int> ignore_ids{};
                // Add inactive robots to the ignore list
                if (robots) {
                    for (const auto& robot : robots->robots) {
                        if (robot.teammate && !robot.purpose.active) {
                            ignore_ids.push_back(robot.purpose.player_id);
                        }
                    }
                }

                // The ball is in the defending third, so we can play
                const unsigned int closest_to_ball =
                    robots ? utility::strategy::closest_to_ball_on_team(ball->rBWw,
                                                                        *robots,
                                                                        field.Hfw,
                                                                        sensors.Hrw,
                                                                        cfg.equidistant_threshold,
                                                                        global_config.player_id,
                                                                        ignore_ids)
                           : global_config.player_id;
                bool is_closest = closest_to_ball == global_config.player_id;

                // Find who has the ball, if any
                // If there are no robots, use an empty vector
                Who ball_pos = utility::strategy::ball_possession(ball->rBWw,
                                                                  (robots ? *robots : Robots{}),
                                                                  field.Hfw,
                                                                  sensors.Hrw,
                                                                  cfg.ball_threshold,
                                                                  cfg.equidistant_threshold,
                                                                  global_config.player_id,
                                                                  ignore_ids);
                // We it's in the defending third, and we are closest to the ball, defend the goals!
                if (is_closest && !penalty) {
                    log<DEBUG>("Goalie is attacking.");
                    emit<Task>(std::make_unique<Attack>(ball_pos));
                    return;
                }

                if (is_closest && penalty) {
                    log<DEBUG>("Goalie is in the best position to take the penalty, readying attack.");
                    emit<Task>(std::make_unique<ReadyAttack>());
                }

                // We are not the closest, but the ball is in the defending third, so we should defend the goals.
                // To do this we stay within the goals on the goal line, but in the spot closest to the ball
                // Clamp the y position to the goal line
                double y_position = std::clamp(rBFf.y(), -fd.dimensions.goal_width / 2.0, fd.dimensions.goal_width / 2.0);
                Eigen::Vector3d rPFf(fd.dimensions.field_length / 2.0, y_position, 0.0);
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
                Eigen::Vector3d rPFf(fd.dimensions.field_length / 2.0, 0.0, 0.0);
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
