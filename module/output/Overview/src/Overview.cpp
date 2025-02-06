/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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
#include "Overview.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"
#include "message/support/GlobalConfig.hpp"
#include "message/support/nusight/Overview.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Goal.hpp"

namespace module::output {

    using extension::Configuration;
    using message::input::GameState;
    using message::input::Image;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::skill::Kick;
    using message::skill::Walk;
    using message::support::GlobalConfig;
    using NUClear::message::CommandLineArguments;

    using LocalisationBall = message::localisation::Ball;
    using VisionBalls      = message::vision::Balls;
    using VisionGoals      = message::vision::Goals;
    using OverviewMsg      = message::support::nusight::Overview;

    Overview::Overview(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Overview.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file Overview.yaml

            // clang-format off
            auto lvl = cfg["log_level"].as<std::string>();
            if (lvl == "TRACE") { this->log_level = TRACE; }
            else if (lvl == "DEBUG") { this->log_level = DEBUG; }
            else if (lvl == "INFO") { this->log_level = INFO; }
            else if (lvl == "WARN") { this->log_level = WARN; }
            else if (lvl == "ERROR") { this->log_level = ERROR; }
            else if (lvl == "FATAL") { this->log_level = FATAL; }
            // clang-format on
        });


        on<Every<2, Per<std::chrono::seconds>>,
           Optional<With<GlobalConfig>>,
           Optional<With<CommandLineArguments>>,
           Optional<With<Sensors>>,
           Optional<With<Field>>,
           Optional<With<LocalisationBall>>,
           Optional<With<Kick>>,
           Optional<With<GameState>>,
           Optional<With<Walk>>,
           Single,
           Priority::LOW>()
            .then([this](const std::shared_ptr<const GlobalConfig>& global,
                         const std::shared_ptr<const CommandLineArguments>& cli,
                         const std::shared_ptr<const Sensors>& sensors,
                         const std::shared_ptr<const Field>& field,
                         const std::shared_ptr<const LocalisationBall>& loc_ball,
                         const std::shared_ptr<const Kick>& kick,
                         const std::shared_ptr<const GameState>& game_state,
                         const std::shared_ptr<const Walk>& walk) {
                auto msg = std::make_unique<OverviewMsg>();

                // Set properties
                msg->timestamp = NUClear::clock::now();
                msg->robot_id  = global ? global->player_id : 0;
                msg->role_name = cli ? cli->at(0) : "";
                msg->battery   = sensors ? sensors->battery : 0;
                msg->voltage   = sensors ? sensors->voltage : 0;

                if (sensors) {
                    // Get our world transform
                    Eigen::Isometry3d Htw(sensors->Htw);

                    // If we have field information
                    if (field) {
                        // Transform the field state into Hfw
                        Eigen::Isometry3d Hfw = Eigen::Isometry3d(field->Hfw);

                        // Get our torso in field space
                        Eigen::Isometry3d Hft = Hfw * Htw.inverse();
                        Eigen::Vector3d rTFf  = Hft.translation();

                        // Store our position from field to torso
                        msg->robot_position =
                            Eigen::Vector3f(rTFf.x(), rTFf.y(), Hft.rotation().matrix().eulerAngles(0, 1, 2).z());
                        msg->robot_position_covariance = field->covariance.cast<float>();

                        if (loc_ball) {
                            // Get our ball in field space
                            Eigen::Vector3d rBWw = loc_ball->rBWw;
                            Eigen::Vector3d rBFf = Hfw * rBWw;

                            // Store our position from field to ball
                            msg->ball_position            = Eigen::Vector2f(rBFf.x(), rBFf.y());
                            msg->ball_position_covariance = loc_ball->covariance.block(0, 0, 2, 2).cast<float>();
                        }
                    }
                }

                if (kick) {
                    msg->kick_target = kick->target.cast<float>().head<2>();
                }

                // Set our game mode properties
                msg->game_mode      = game_state ? game_state->mode : GameState::Mode(0);
                msg->game_phase     = game_state ? game_state->phase : GameState::Phase(0);
                msg->penalty_reason = game_state ? game_state->self.penalty_reason : GameState::PenaltyReason(0);

                // Set our last seen times
                msg->last_camera_image = last_camera_image;
                msg->last_camera_image = last_seen_ball;
                msg->last_camera_image = last_seen_goal;

                // Set our walk command
                if (walk) {
                    msg->walk_command = walk->velocity_target.cast<float>();
                }
                else {
                    msg->walk_command = Eigen::Vector3f::Zero();
                }

                emit(msg);
            });


        on<Trigger<Image>, Single, Priority::LOW>().then([this] { last_camera_image = NUClear::clock::now(); });

        on<Trigger<VisionBalls>, Single, Priority::LOW>().then([this](const VisionBalls& balls) {
            if (!balls.balls.empty()) {
                last_seen_ball = NUClear::clock::now();
            }
        });

        on<Trigger<VisionGoals>, Single, Priority::LOW>().then([this](const VisionGoals& goals) {
            if (!goals.goals.empty()) {
                last_seen_goal = NUClear::clock::now();
            }
        });
    }

}  // namespace module::output
