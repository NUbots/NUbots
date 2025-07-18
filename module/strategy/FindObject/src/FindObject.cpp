/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of         //         // Initialise state machine when Search starts
        on<Start<Search>>().then([this] {
            log<DEBUG>("Starting Search - initialising state machine to TURNING_ON_SPOT");
            current_state = SearchState::TURNING_ON_SPOT;
            patrol_target = 0;  // Reset patrol target
            state_start_time = NUClear::clock::now(); // Record when this state started
            initial_heading_saved = false; // Reset heading tracking
        });lize state machine when Search starts
        on<Start<Search>>().then([this] {
            log<DEBUG>("Starting Search - initializing state machine to TURNING_ON_SPOT");
            current_state = SearchState::TURNING_ON_SPOT;
            patrol_target = 0; // Reset patrol target
            state_start_time = NUClear::clock::now(); // Record when this state started
            initial_heading_saved = false; // Reset heading tracking
        });bots codebase.
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
#include "FindObject.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/LookAround.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/purpose/Purpose.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::GameState;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::planning::LookAround;
    using message::planning::TurnOnSpot;
    using message::purpose::Purpose;
    using message::purpose::SoccerPosition;
    using message::strategy::FindBall;
    using message::strategy::Search;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;
    using utility::math::euler::pos_rpy_to_transform;
    using utility::support::Expression;

    FindObject::FindObject(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FindObject.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FindObject.yaml
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["ball_search_timeout"].as<double>()));
            cfg.border_threshold         = config["border_threshold"].as<double>();
            cfg.centre_move_ratio        = config["centre_move_ratio"].as<double>();
            cfg.centre_reached_threshold = config["centre_reached_threshold"].as<double>();
            cfg.patrol_reached_threshold = config["patrol_reached_threshold"].as<double>();
            cfg.turn_duration            = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["turn_duration"].as<double>()));
        });

        on<Provide<FindBall>,
           Uses<Search>,
           Optional<With<Ball>>,
           Optional<With<GlobalConfig>>,
           Optional<With<GameState>>>()
            .then([this](const Uses<Search>& search,
                         const std::shared_ptr<const Ball>& ball,
                         const std::shared_ptr<const GlobalConfig>& global_config,
                         const std::shared_ptr<const GameState>& game_state) {
                if (ball == nullptr || (NUClear::clock::now() - ball->time_of_measurement) > cfg.ball_search_timeout) {
                    // Emit purpose information if we are searching for the ball
                    if (global_config) {
                        emit(std::make_unique<Purpose>(global_config->player_id,
                                                       SoccerPosition::UNKNOWN,
                                                       true,
                                                       false,
                                                       game_state
                                                           ? game_state->team.team_colour
                                                           : GameState::TeamColour(GameState::TeamColour::UNKNOWN)));
                    }

                    if (search.run_state == RunState::NO_TASK) {
                        // Start searching by emitting Search task
                        log<INFO>("Searching for the ball.");
                        emit<Task>(std::make_unique<Search>());
                    }
                    else {
                        emit<Task>(std::make_unique<Continue>());
                    }
                }
            });


        // Initialise state machine when Search starts
        on<Start<Search>>().then([this] {
            log<DEBUG>("Starting Search - initialising state machine to TURNING_ON_SPOT");
            current_state         = SearchState::TURNING_ON_SPOT;
            patrol_target         = 0;
            state_start_time      = NUClear::clock::now();
            initial_heading_saved = false;
        });

        on<Provide<Search>, Every<1, Per<std::chrono::seconds>>, With<Field>, With<Sensors>, With<FieldDescription>>()
            .then([this](const Field& field, const Sensors& sensors, const FieldDescription& field_desc) {
                // Conduct a head search while searching for the ball
                emit<Task>(std::make_unique<LookAround>());

                // Get robot position and check border
                Eigen::Vector3d rRFf = (field.Hfw * sensors.Hrw.inverse()).translation();
                Eigen::Matrix3d Rfr  = (sensors.Hrw * field.Hfw.inverse()).inverse().rotation();
                bool near_border =
                    (std::abs(rRFf.x()) > (field_desc.dimensions.field_length / 2.0 - cfg.border_threshold))
                    || (std::abs(rRFf.y()) > (field_desc.dimensions.field_width / 2.0 - cfg.border_threshold));

                if (near_border) {
                    log<DEBUG>("Near border, moving to centre");
                    current_state         = SearchState::MOVING_TO_CENTRE;
                    state_start_time      = NUClear::clock::now();
                    initial_heading_saved = false;
                }

                switch (current_state) {
                    case SearchState::TURNING_ON_SPOT: {
                        log<DEBUG>("Turning on spot to find the ball");

                        emit<Task>(std::make_unique<TurnOnSpot>(false));

                        // Check if we've been turning for the configured duration
                        if ((NUClear::clock::now() - state_start_time) > cfg.turn_duration) {
                            current_state         = SearchState::MOVING_TO_CENTRE;
                            state_start_time      = NUClear::clock::now();
                            initial_heading_saved = false;
                        }
                        break;
                    }

                    case SearchState::MOVING_TO_CENTRE: {
                        log<DEBUG>("Moving to centre to find the ball");
                        // Get heading towards the centre of the field
                        double heading = std::atan2(Rfr(1, 0), Rfr(0, 0));
                        emit<Task>(std::make_unique<WalkToFieldPosition>(
                            pos_rpy_to_transform(Eigen::Vector3d(Eigen::Vector3d::Zero()),
                                                 Eigen::Vector3d(0.0, 0.0, heading)),
                            false));

                        if (rRFf.norm() < cfg.centre_reached_threshold) {
                            current_state    = SearchState::PATROLLING;
                            state_start_time = NUClear::clock::now();
                        }
                        break;
                    }

                    case SearchState::PATROLLING: {
                        log<DEBUG>("Patrolling to find the ball");
                        double half_length = field_desc.dimensions.field_length / 2.0;
                        double half_width  = field_desc.dimensions.field_width / 2.0;

                        std::vector<Eigen::Vector3d> patrol_points = {{half_length * 0.5, half_width * 0.5, 0},
                                                                      {half_length * 0.5, -half_width * 0.5, 0},
                                                                      {-half_length * 0.5, -half_width * 0.5, 0},
                                                                      {-half_length * 0.5, half_width * 0.5, 0}};

                        Eigen::Vector3d target = patrol_points[patrol_target % patrol_points.size()];

                        if ((rRFf - target).norm() < cfg.patrol_reached_threshold) {
                            patrol_target++;
                            target = patrol_points[patrol_target % patrol_points.size()];
                        }

                        Eigen::Vector3d next_target = patrol_points[(patrol_target + 1) % patrol_points.size()];
                        double yaw = std::atan2((next_target - target).y(), (next_target - target).x());

                        emit<Task>(std::make_unique<WalkToFieldPosition>(
                            pos_rpy_to_transform(target, Eigen::Vector3d(0, 0, yaw)),
                            false));
                        break;
                    }
                }
            });
    }

}  // namespace module::strategy
