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

#include "SimpleWalkPathPlanner.hpp"

#include <Eigen/Geometry>
#include <cmath>

#include "extension/Configuration.hpp"

#include "message/behaviour/KickPlan.hpp"
#include "message/behaviour/MotionCommand.hpp"
#include "message/behaviour/Subsumption.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/motion/KickCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/localisation/transform.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/math/coordinates.hpp"


namespace module::behaviour::planning {

    using extension::Configuration;

    using message::behaviour::KickPlan;
    using message::behaviour::MotionCommand;
    using message::behaviour::WantsToKick;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::KickFinished;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::motion::WalkStopped;
    using message::support::FieldDescription;
    using VisionBalls = message::vision::Balls;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;
    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::localisation::fieldStateToTransform3D;
    using utility::nusight::graph;
    using utility::math::coordinates::sphericalToCartesian;

    SimpleWalkPathPlanner::SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , latestCommand(utility::behaviour::StandStill())
        , subsumptionId(size_t(this) * size_t(this) - size_t(this))
        , currentTargetPosition(Eigen::Vector2d::Zero())
        , currentTargetHeading(Eigen::Vector2d::Zero())
        , targetHeading(Eigen::Vector2d::Zero(), KickPlan::KickType::SCRIPTED)
        , timeBallLastSeen(NUClear::clock::now()) {

        // do a little configurating
        on<Configuration>("SimpleWalkPathPlanner.yaml").then([this](const Configuration& file) {
            // TODO(KipHamiltons): Make these all consistent with the other files. We don't use .config anywhere else
            log_level = file.config["log_level"].as<NUClear::LogLevel>();

            turnSpeed            = file.config["turnSpeed"].as<float>();
            forwardSpeed         = file.config["forwardSpeed"].as<float>();
            walkToReadySpeedX     = file.config["walkToReadySpeedX"].as<float>();
            walkToReadySpeedY     = file.config["walkToReadySpeedY"].as<float>();
            walkToReadyRotation    = file.config["walkToReadyRotation"].as<float>();
            rotateSpeed     = file.config["rotateSpeed"].as<float>();
            rotateSpeedX     = file.config["rotateSpeedX"].as<float>();
            rotateSpeedY     = file.config["rotateSpeedY"].as<float>();
            sideSpeed            = file.config["sideSpeed"].as<float>();
            a                    = file.config["a"].as<float>();
            b                    = file.config["b"].as<float>();
            search_timeout       = file.config["search_timeout"].as<float>();
            robot_ground_space   = file.config["robot_ground_space"].as<bool>();
            ball_approach_dist   = file.config["ball_approach_dist"].as<float>();
            slowdown_distance    = file.config["slowdown_distance"].as<float>();
            useLocalisation      = file.config["useLocalisation"].as<bool>();
            slow_approach_factor = file.config["slow_approach_factor"].as<float>();

            emit(std::make_unique<WantsToKick>(false));
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            subsumptionId,
            "Simple Walk Path Planner",
            {
                // Limb sets required by the walk engine:
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
            },
            [this](const std::set<LimbID>& givenLimbs) {
                if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                    // Enable the walk engine.
                    emit<Scope::DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                }
            },
            [this](const std::set<LimbID>& takenLimbs) {
                if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                    // Shut down the walk engine, since we don't need it right now.
                    emit<Scope::DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
                }
            },
            [this](const std::set<ServoID>&) {
                // nothing
            }}));

        on<Trigger<WalkStopped>>().then([this] {
            emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {0, 0}}));
        });


        on<Trigger<VisionBalls>, With<Sensors>>().then([this](const VisionBalls& balls, const Sensors& sensors) {
            if (balls.balls.size() > 0) {
                Eigen::Vector3f srBCc = balls.balls[0].measurements[0].srBCc;
                srBCc.x() = 1.0/srBCc.x();
                Eigen::Affine3f Htc(sensors.Htw.cast<float>() * balls.Hcw.inverse().cast<float>());
                rBTt = Htc * Eigen::Vector3f(sphericalToCartesian(srBCc));
                log("rBTt: ", rBTt.x(),rBTt.y(),rBTt.z());
                timeBallLastSeen = NUClear::clock::now();
            }
        });

        // Freq should be equal to the main loop in soccer strategy
        on<Every<100, Per<std::chrono::seconds>>,
           With<Ball>,
           With<Field>,
           With<Sensors>,
           With<WantsToKick>,
           With<KickPlan>,
           With<FieldDescription>,
           Sync<SimpleWalkPathPlanner>>()
            .then([this](const Ball& ball,
                         const Field& field,
                         const Sensors& sensors,
                         const WantsToKick& wantsTo,
                         const KickPlan& kickPlan,
                         const FieldDescription& fieldDescription) {
                if (wantsTo.kick) {
                    emit(std::make_unique<StopCommand>(subsumptionId));
                    return;
                }

                if (latestCommand.type == message::behaviour::MotionCommand::Type::STAND_STILL) {
                    log("stop command emitted");
                    emit(std::make_unique<StopCommand>(subsumptionId));
                    return;
                }
                else if (latestCommand.type == message::behaviour::MotionCommand::Type::ABSOLUTE_STOP) {
                    log("absolute stop");
                    emit<Scope::DIRECT>(std::make_unique<DisableWalkEngineCommand>(subsumptionId));
                    return;
                }
                else if (latestCommand.type == message::behaviour::MotionCommand::Type::STOP_ABSOLUTE_STOP) {
                    emit<Scope::DIRECT>(std::make_unique<EnableWalkEngineCommand>(subsumptionId));
                    return;
                }
                else if (latestCommand.type == message::behaviour::MotionCommand::Type::DIRECT_COMMAND) {
                    // emit(std::make_unique<EnableWalkEngineCommand>(subsumptionId));
                    // TO DO, change to Bezier stuff
                    std::unique_ptr<WalkCommand> command =
                        std::make_unique<WalkCommand>(subsumptionId, latestCommand.walk_command);
                    emit(std::move(command));
                    emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
                    return;
                }   else if (latestCommand.type == message::behaviour::MotionCommand::Type::WALK_TO_READY_POSITION) {
                    // log("Walk to ready");
                    std::unique_ptr<WalkCommand> command =
                        std::make_unique<WalkCommand>(subsumptionId,
                                                  Eigen::Vector3d(walkToReadySpeedX, walkToReadySpeedY, walkToReadyRotation));
                    emit(std::move(command));
                    emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
                    return;
                } else if (latestCommand.type == message::behaviour::MotionCommand::Type::ROTATE_ON_SPOT) {
                    log("Rotate on spot");
                    std::unique_ptr<WalkCommand> command =
                        std::make_unique<WalkCommand>(subsumptionId,
                                                  Eigen::Vector3d(rotateSpeedX, rotateSpeedY, rotateSpeed));
                    emit(std::move(command));
                    emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
                    return;
                } else if (latestCommand.type == message::behaviour::MotionCommand::Type::WALK_TO_BALL) {
                    std::unique_ptr<WalkCommand> command;
                    // if(std::fabs(rBTt.y()) < 1){
                    log("Walk to rBTt: ", rBTt.x(),rBTt.y(),rBTt.z());
                    Eigen::Vector3f unit_vector_to_ball = rBTt / rBTt.norm();
                    Eigen::Vector3f velocity_vector = 0.03 * unit_vector_to_ball;
                    log("Walk command: ", velocity_vector.x(),velocity_vector.y(),velocity_vector.z());
                    float heading_angle = std::atan2(velocity_vector.y(), velocity_vector.x());
                    command =
                        std::make_unique<WalkCommand>(subsumptionId,
                                                    Eigen::Vector3d(velocity_vector.x(), velocity_vector.y(), 0.9*heading_angle));
                    // } else {
                    //     log("Rotate on spot");
                    //     if(rBTt.y() < 0) {
                    //         rotateSpeed = -rotateSpeed;
                    //     }
                    //     command =
                    //         std::make_unique<WalkCommand>(subsumptionId,
                    //                                 Eigen::Vector3d(rotateSpeedX, rotateSpeedY, rotateSpeed));
                    // }
                    emit(std::move(command));
                    emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
                    return;
                }
                // emit(std::make_unique<EnableWalkEngineCommand>(subsumptionId));

                // Eigen::Affine3d Htw(sensors.Htw);
                // // Get ball position
                // log("ball position x [m]:", ball.position.x());
                // log("ball position y [m]::", ball.position.y());
                // Eigen::Vector3d rBWw(ball.position.x(), ball.position.y(), fieldDescription.ball_radius);
                // // Transform ball position to torso space
                // Eigen::Vector3d rBTt = Htw * rBWw;
                // // Get unit vector to ball in torso space
                // Eigen::Vector3d unit_vector_to_ball = rBTt / rBTt.norm();
                // // Calculate velocity vector to ball by scalling unit vector by forward velocity
                // Eigen::Vector3d velocity_vector = forwardSpeed * unit_vector_to_ball;
                // // Calculate heading angle
                // float heading_angle = std::atan2(rBTt.y(), rBTt.x());
                // float max_turn_speed = 0.1;
                // heading_angle = std::max(max_turn_speed,heading_angle);

                // float scale_rotation = 1.1;

                // std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>(
                //     subsumptionId,
                //     Eigen::Vector3d(velocity_vector.x(), velocity_vector.y(), 0));
                // // log("x velocity command [m/s]:", velocity_vector.x());
                // // log("y velocity command [m/s]:", velocity_vector.y());
                // // log("heading angle [rad]:", heading_angle);

                // emit(std::move(command));
                // emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
            });

        on<Trigger<MotionCommand>, Sync<SimpleWalkPathPlanner>>().then([this](const MotionCommand& cmd) {
            // save the plan
            latestCommand = cmd;
        });

        on<Trigger<MotionCommand>>().then([this](const MotionCommand& cmd) {
            // TODO(cameron) unhack
            if (cmd.type == message::behaviour::MotionCommand::Type::ABSOLUTE_STOP) {
                emit<Scope::DIRECT>(std::make_unique<DisableWalkEngineCommand>(subsumptionId));
            }
            else if (latestCommand.type == message::behaviour::MotionCommand::Type::STOP_ABSOLUTE_STOP) {
                emit<Scope::DIRECT>(std::make_unique<EnableWalkEngineCommand>(subsumptionId));
            }
        });
    }
}  // namespace module::behaviour::planning
