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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "message/support/nusight/Overview.h"

#include "NUsight.h"
#include "message/behaviour/Behaviour.h"
#include "message/behaviour/KickPlan.h"
#include "message/behaviour/WalkPath.h"
#include "message/input/GameState.h"
#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/localisation/Ball.h"
#include "message/localisation/Field.h"
#include "message/motion/WalkCommand.h"
#include "message/support/GlobalConfig.h"
#include "message/vision/Ball.h"
#include "message/vision/Goal.h"
#include "utility/localisation/transform.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/eigen_armadillo.h"

/**
 * @author Monica Olejniczak
 */
namespace module {
namespace support {

    using message::behaviour::Behaviour;
    using message::behaviour::KickPlan;
    using message::behaviour::WalkPath;
    using message::input::GameState;
    using message::input::Image;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::support::GlobalConfig;
    using message::support::nusight::Overview;
    using NUClear::message::CommandLineArguments;
    using LocalisationBall = message::localisation::Ball;
    using VisionBalls      = message::vision::Balls;
    using VisionGoals      = message::vision::Goals;
    using message::motion::WalkCommand;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform3D;

    /**
     * @brief Provides triggers to send overview information over the network using the overview
     * instance variable.
     */
    void NUsight::provideOverview() {

        handles["overview"].push_back(
            on<Every<2, Per<std::chrono::seconds>>,
               Optional<With<GlobalConfig>>,
               Optional<With<CommandLineArguments>>,
               Optional<With<Sensors>>,
               Optional<With<Behaviour::State>>,
               Optional<With<Field>>,
               Optional<With<LocalisationBall>>,
               Optional<With<KickPlan>>,
               Optional<With<GameState>>,
               Optional<With<WalkPath>>,
               Optional<With<WalkCommand>>,
               Single,
               Priority::LOW>()
                .then([this](std::shared_ptr<const GlobalConfig> global,
                             std::shared_ptr<const CommandLineArguments> cli,
                             std::shared_ptr<const Sensors> sensors,
                             std::shared_ptr<const Behaviour::State> behaviour_state,
                             std::shared_ptr<const Field> field,
                             std::shared_ptr<const LocalisationBall> loc_ball,
                             std::shared_ptr<const KickPlan> kick_plan,
                             std::shared_ptr<const GameState> game_state,
                             std::shared_ptr<const WalkPath> walk_path,
                             std::shared_ptr<const WalkCommand> walk_command) {
                    auto msg = std::make_unique<Overview>();

                    // Set properties
                    msg->timestamp       = NUClear::clock::now();
                    msg->robot_id        = global ? global->playerId : 0;
                    msg->role_name       = cli ? cli->at(0) : "";
                    msg->battery         = sensors ? sensors->battery : 0;
                    msg->voltage         = sensors ? sensors->voltage : 0;
                    msg->behaviour_state = behaviour_state ? msg->behaviour_state : Behaviour::State(0);

                    if (sensors) {
                        // Get our world transform
                        Transform3D Htw(convert(sensors->Htw));

                        // If we have field information
                        if (field) {
                            Transform3D Hfw = utility::localisation::fieldStateToTransform3D(convert(field->position));

                            // Get our torso in field space
                            Transform3D Hft = Hfw * Htw.i();
                            arma::vec3 rTFf = Hft.translation();

                            // Get the rotation
                            Rotation3D Rft = Hft.rotation();

                            // Store our position from field to torso
                            msg->robot_position            = Eigen::Vector3f(rTFf[0], rTFf[1], Rft.yaw());
                            msg->robot_position_covariance = field->covariance.cast<float>();

                            if (loc_ball) {
                                // Get our ball in field space
                                arma::vec2 rBWw_2d = convert(loc_ball->position);
                                arma::vec4 rBWw    = {rBWw_2d[0], rBWw_2d[1], 0, 1};
                                arma::vec4 rBFf    = Hfw * rBWw;

                                // Store our position from field to ball
                                msg->ball_position            = Eigen::Vector2f(rBFf[0], rBFf[1]);
                                msg->ball_position_covariance = loc_ball->covariance.cast<float>();
                            }
                        }

                        if (walk_path) {
                            // Set our walk path plan
                            if (walk_path) {
                                for (const auto& state : walk_path->states) {
                                    msg->walk_path_plan.push_back(state);
                                }
                            }
                        }
                    }

                    if (kick_plan) {
                        msg->kick_target = kick_plan->target.cast<float>();
                    }

                    // Set our game mode properties
                    msg->game_mode  = game_state ? game_state->data.mode : GameState::Data::Mode(0);
                    msg->game_phase = game_state ? game_state->data.phase : GameState::Data::Phase(0);
                    msg->penalty_reason =
                        game_state ? game_state->data.self.penalty_reason : GameState::Data::PenaltyReason(0);

                    // Set our last seen times
                    msg->last_camera_image = last_camera_image;
                    msg->last_camera_image = last_seen_ball;
                    msg->last_camera_image = last_seen_goal;

                    // Set our walk command
                    if (walk_command) {
                        msg->walk_command = walk_command->command.cast<float>();
                    }
                    else {
                        msg->walk_command = Eigen::Vector3f::Zero();
                    }

                    emit<Scope::NETWORK>(msg, "nusight", false);
                }));


        handles["overview"].push_back(
            on<Trigger<Image>, Single, Priority::LOW>().then([this] { last_camera_image = NUClear::clock::now(); }));

        handles["overview"].push_back(
            on<Trigger<VisionBalls>, Single, Priority::LOW>().then([this](const VisionBalls& balls) {
                if (!balls.balls.empty()) {
                    last_seen_ball = NUClear::clock::now();
                }
            }));

        handles["overview"].push_back(
            on<Trigger<VisionGoals>, Single, Priority::LOW>().then([this](const VisionGoals& goals) {
                if (!goals.goals.empty()) {
                    last_seen_goal = NUClear::clock::now();
                }
            }));
    }
}  // namespace support
}  // namespace module
