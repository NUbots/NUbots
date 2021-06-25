/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "KickPlanner.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "extension/Configuration.hpp"

#include "message/behaviour/KickPlan.hpp"
#include "message/behaviour/ServoCommand.hpp"
#include "message/input/GameState.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/motion/KinematicsModel.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/localisation/transform.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/motion/InverseKinematics.hpp"
#include "utility/nusight/NUhelpers.hpp"


namespace module::behaviour::planning {

    using extension::Configuration;

    using message::behaviour::KickPlan;
    using message::behaviour::WantsToKick;
    using message::input::GameState;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::motion::IKKickParams;
    using message::motion::KickCommand;
    using message::motion::KickCommandType;
    using message::motion::KickPlannerConfig;
    using message::motion::KickScriptCommand;
    using message::motion::KinematicsModel;
    using message::platform::ButtonMiddleDown;
    using message::support::FieldDescription;

    using KickType      = message::behaviour::KickPlan::KickType;
    using PenaltyReason = message::input::GameState::Data::PenaltyReason;
    using Phase         = message::input::GameState::Data::Phase;
    using VisionBall    = message::vision::Ball;
    using VisionBalls   = message::vision::Balls;

    using utility::input::LimbID;
    using utility::localisation::fieldStateToTransform3D;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::nusight::graph;

    KickPlanner::KickPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , cfg()
        , ballLastSeen(std::chrono::seconds(0))
        , lastTimeValid(NUClear::clock::now()) {


        on<Configuration>("KickPlanner.yaml").then([this](const Configuration& config) {
            cfg.max_ball_distance        = config["max_ball_distance"].as<float>();
            cfg.kick_corridor_width      = config["kick_corridor_width"].as<float>();
            cfg.seconds_not_seen_limit   = config["seconds_not_seen_limit"].as<float>();
            cfg.kick_forward_angle_limit = config["kick_forward_angle_limit"].as<float>();
            emit(std::make_unique<KickPlannerConfig>(cfg));
            emit(std::make_unique<WantsToKick>(false));
        });

        on<Trigger<VisionBalls>>().then([this](const VisionBalls& balls) {
            if (balls.balls.size() > 0) {
                ballLastSeen = NUClear::clock::now();
            }
        });

        on<Trigger<ButtonMiddleDown>, Single>().then([this] { forcePlaying = true; });

        on<Trigger<Ball>,
           With<Field>,
           With<FieldDescription>,
           With<KickPlan>,
           With<Sensors>,
           Optional<With<GameState>>>()
            .then([this](const Ball& ball,
                         const Field& field,
                         const FieldDescription& fd,
                         const KickPlan& kickPlan,
                         const Sensors& sensors,
                         std::shared_ptr<const GameState> gameState) {
                // Get time since last seen ball
                auto now = NUClear::clock::now();
                double secondsSinceLastSeen =
                    std::chrono::duration_cast<std::chrono::microseconds>(now - ballLastSeen).count() * 1e-6;

                // Compute target in robot coords
                // Eigen::Vector3d kickTarget = Eigen::Vector3d::UnitX(); //Kick forwards
                Eigen::Affine2d position = Eigen::Affine2d(field.position);
                Eigen::Affine3d Hfw;
                Hfw.translation() = Eigen::Vector3d(position.translation().x(), position.translation().y(), 0);
                Hfw.linear() =
                    Eigen::AngleAxisd(Eigen::Rotation2Dd(position.rotation()).angle(), Eigen::Vector3d::UnitZ())
                        .toRotationMatrix();

                Eigen::Affine3d Htw(sensors.Htw);
                Eigen::Vector3d ballPosition =
                    Htw * Eigen::Vector3d(ball.position.x(), ball.position.y(), fd.ball_radius);

                // Transform target from field to torso space
                Eigen::Affine3d Htf        = Htw * Hfw.inverse();
                Eigen::Vector3d kickTarget = Htf * Eigen::Vector3d(kickPlan.target.x(), kickPlan.target.y(), 0.0);
                float KickAngle            = std::fabs(std::atan2(kickTarget.y(), kickTarget.x()));

                // log("KickPlan target global",kickPlan. target.transpose());
                // log("Target of Kick", kickTarget.transpose());
                // log("KickAngle", KickAngle);

                // Check whether to kick
                // log("kickTarget",kickTarget.t());
                // log("KickAngle",KickAngle);
                // log("ballPosition",ballPosition);
                // log("secondsSinceLastSeen",secondsSinceLastSeen);
                bool correctState = true;
                if (gameState) {
                    // If we are playing with GameController, only kick if we are in the playing state, are not
                    // penalised and are not in ready state
                    correctState = gameState->data.phase == Phase::PLAYING
                                   && gameState->data.self.penalty_reason == PenaltyReason::UNPENALISED
                                   && gameState->data.phase != Phase::READY;
                }
                bool kickIsValid = kickValid(ballPosition);
                if (kickIsValid) {
                    lastTimeValid = now;
                }
                float timeSinceValid = (now - lastTimeValid).count() * (1 / double(NUClear::clock::period::den));

                // log("kick checks",secondsSinceLastSeen < cfg.seconds_not_seen_limit
                //     , kickIsValid
                //     , KickAngle < cfg.kick_forward_angle_limit);
                if (secondsSinceLastSeen < cfg.seconds_not_seen_limit && kickIsValid && (correctState || forcePlaying)
                    && KickAngle < cfg.kick_forward_angle_limit) {

                    switch (kickPlan.kick_type.value) {
                        case KickType::IK_KICK:
                            // NUClear::log("ik_kick");
                            if (ballPosition.y() > 0.0) {
                                emit(std::make_unique<KickCommand>(KickCommand(Eigen::Vector3d(0.1, 0.04, 0),
                                                                               Eigen::Vector3d::UnitX(),
                                                                               KickCommandType::NORMAL)));
                                emit(std::make_unique<WantsToKick>(true));
                            }
                            else {
                                emit(std::make_unique<KickCommand>(KickCommand(Eigen::Vector3d(0.1, -0.04, 0),
                                                                               Eigen::Vector3d::UnitX(),
                                                                               KickCommandType::NORMAL)));
                                emit(std::make_unique<WantsToKick>(true));
                            }
                            break;
                        case KickType::SCRIPTED:
                            // NUClear::log("scripted");
                            if (ballPosition.y() > 0.0) {
                                emit(std::make_unique<KickScriptCommand>(
                                    KickScriptCommand(LimbID::LEFT_LEG, KickCommandType::NORMAL)));
                                emit(std::make_unique<WantsToKick>(true));
                                ;
                            }
                            else {
                                emit(std::make_unique<KickScriptCommand>(
                                    KickScriptCommand(LimbID::RIGHT_LEG, KickCommandType::NORMAL)));
                                emit(std::make_unique<WantsToKick>(true));
                                ;
                            }
                            break;
                        default: throw new std::runtime_error("KickPlanner: Invalid KickType");
                    }
                }
                else if (secondsSinceLastSeen > cfg.seconds_not_seen_limit
                         || timeSinceValid > cfg.seconds_not_seen_limit) {
                    emit(std::make_unique<WantsToKick>(WantsToKick(false)));
                }
            });
    }


    bool KickPlanner::kickValid(const Eigen::Vector3d& ballPos) {
        return (ballPos.x() > 0.0) && (ballPos.x() < cfg.max_ball_distance)
               && (std::fabs(ballPos.y()) < cfg.kick_corridor_width * 0.5);
    }
}  // namespace module::behaviour::planning
