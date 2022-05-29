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
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::motion::WalkStopped;
    using message::support::FieldDescription;
    using VisionBalls = message::vision::Balls;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;
    using utility::input::LimbID;
    using utility::input::ServoID;

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

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
            RegisterAction{subsumptionId,
                           "Simple Walk Path Planner",
                           {
                               // Limb sets required by the walk engine:
                               std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                               std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
                           },
                           [this](const std::set<LimbID>& givenLimbs) {
                               if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                                   // Enable the walk engine.
                                   emit<Scope::DIRECT>(std::make_unique<EnableWalkEngineCommand>(subsumptionId));
                               }
                           },
                           [this](const std::set<LimbID>& takenLimbs) {
                               if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                                   // Shut down the walk engine, since we don't need it right now.
                                   emit<Scope::DIRECT>(std::make_unique<DisableWalkEngineCommand>(subsumptionId));
                               }
                           },
                           [](const std::set<ServoID>& /*unused*/) {
                               // nothing
                           }}));

        on<Trigger<WalkStopped>>().then([this] {
            emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {0, 0}}));
        });


        on<Trigger<VisionBalls>>().then([this](const VisionBalls& balls) {
            if (!balls.balls.empty()) {
                timeBallLastSeen = NUClear::clock::now();
            }
        });

        // Freq should be equal to the main loop in soccer strategy
        on<Every<30, Per<std::chrono::seconds>>,
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
                    emit(std::make_unique<StopCommand>(subsumptionId));
                    return;
                }
                if (latestCommand.type == message::behaviour::MotionCommand::Type::DIRECT_COMMAND) {
                    // TO DO, change to Bezier stuff
                    std::unique_ptr<WalkCommand> command =
                        std::make_unique<WalkCommand>(subsumptionId, latestCommand.walk_command);
                    emit(std::move(command));
                    emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
                    return;
                }

                Eigen::Affine3d Htw(sensors.Htw);

                auto now = NUClear::clock::now();
                float timeSinceBallSeen =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now - timeBallLastSeen).count()
                    * (1.0f / std::nano::den);


                Eigen::Vector3d rBWw_temp(ball.position.x(), ball.position.y(), fieldDescription.ball_radius);
                rBWw     = timeSinceBallSeen < search_timeout ? rBWw_temp :                         // Place last seen
                           Htw.inverse().linear().leftCols<1>() + Htw.inverse().translation();  // In front of the robot
                position = (Htw * rBWw).head<2>();

                // Hack Planner:
                float headingChange = 0;
                float sideStep      = 0;
                float speedFactor   = 1;
                if (useLocalisation) {

                    // Transform kick target to torso space
                    auto fieldPosition = Eigen::Affine2d(field.position);
                    Eigen::Affine3d Hfw;
                    Hfw.translation() =
                        Eigen::Vector3d(fieldPosition.translation().x(), fieldPosition.translation().y(), 0);
                    Hfw.linear() = Eigen::AngleAxisd(Eigen::Rotation2Dd(fieldPosition.rotation()).angle(),
                                                     Eigen::Vector3d::UnitZ())
                                       .toRotationMatrix();

                    Eigen::Affine3d Htf        = Htw * Hfw.inverse();
                    Eigen::Vector3d kickTarget = Htf * Eigen::Vector3d(kickPlan.target.x(), kickPlan.target.y(), 0);

                    // //approach point:
                    Eigen::Vector2d ballToTarget = (kickTarget.head<2>() - position).normalized();
                    Eigen::Vector2d kick_point   = position - ballToTarget * ball_approach_dist;

                    if (position.norm() > slowdown_distance) {
                        position = kick_point;
                    }
                    else {
                        speedFactor   = slow_approach_factor;
                        headingChange = std::atan2(ballToTarget.y(), ballToTarget.x());
                        sideStep      = 1;
                    }
                }


                float angle = std::atan2(position.y(), position.x()) + headingChange;

                angle = std::min(turnSpeed, std::max(angle, -turnSpeed));


                // Euclidean distance to ball
                float scaleF            = 2.0 / (1.0 + std::exp(-a * std::fabs(position.x()) + b)) - 1.0;
                float scaleF2           = angle / M_PI;
                float finalForwardSpeed = speedFactor * forwardSpeed * scaleF * (1.0 - scaleF2);

                float scaleS         = 2.0 / (1.0 + std::exp(-a * std::fabs(position.y()) + b)) - 1.0;
                float scaleS2        = angle / M_PI;
                float finalSideSpeed = -speedFactor * (float(0.0f < position.y()) - float(position.y() < 0.0))
                                       * sideStep * sideSpeed * scaleS * (1.0 - scaleS2);

                std::unique_ptr<WalkCommand> command =
                    std::make_unique<WalkCommand>(subsumptionId,
                                                  Eigen::Vector3d(finalForwardSpeed, finalSideSpeed, angle));

                emit(std::move(command));
                emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
            });

        on<Trigger<MotionCommand>, Sync<SimpleWalkPathPlanner>>().then([this](const MotionCommand& cmd) {
            // save the plan
            latestCommand = cmd;
        });
    }
}  // namespace module::behaviour::planning
