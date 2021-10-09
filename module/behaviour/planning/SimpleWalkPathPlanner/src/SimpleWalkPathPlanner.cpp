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
#include "utility/math/coordinates.hpp"
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

            maxTurnSpeed         = file.config["maxTurnSpeed"].as<float>();
            minTurnSpeed         = file.config["minTurnSpeed"].as<float>();
            forwardSpeed         = file.config["forwardSpeed"].as<float>();
            sideSpeed            = file.config["sideSpeed"].as<float>();
            rotateSpeedX         = file.config["rotateSpeedX"].as<float>();
            rotateSpeedY         = file.config["rotateSpeedY"].as<float>();
            rotateSpeed          = file.config["rotateSpeed"].as<float>();
            walkToReadySpeedX    = file.config["walkToReadySpeedX"].as<float>();
            walkToReadySpeedY    = file.config["walkToReadySpeedY"].as<float>();
            walkToReadyRotation  = file.config["walkToReadyRotation"].as<float>();
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

        // Used to maintain the time since the ball was last seen for a timeout
        // period (could be implemented outside of the path planner and somewhere
        // else in strategy to call a particular 'style' of path planning that is
        // more likely to find the ball)
        on<Trigger<VisionBalls>, With<Sensors>>().then([this](const VisionBalls& balls, const Sensors& sensors) {
            if (balls.balls.size() > 0) {
                Eigen::Vector3f srBCc = balls.balls[0].measurements[0].srBCc;
                srBCc.x()             = 1.0 / srBCc.x();
                Eigen::Affine3f Htc(sensors.Htw.cast<float>() * balls.Hcw.inverse().cast<float>());
                rBTt = Htc * Eigen::Vector3f(sphericalToCartesian(srBCc));
                log("rBTt: ", rBTt.x(), rBTt.y(), rBTt.z());
                timeBallLastSeen = NUClear::clock::now();
            }
        });

        // Freq should be equal to the main loop in soccer strategy. TODO (Bryce Tuppurainen): Potentially
        // change this value to a define in an included header if this will be used again for added design cohesion if
        // this will be something we change in future
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
                // TODO(Bryce Tuppurainen) Determine if this line is necessary. Could this be
                // integrated into the switch?
                if (wantsTo.kick) {
                    emit(std::make_unique<StopCommand>(subsumptionId));
                    return;
                }

                switch (static_cast<int>(latestCommand.type.value)) {
                    case message::behaviour::MotionCommand::Type::STAND_STILL:
                        emit(std::make_unique<StopCommand>(subsumptionId));
                        return;

                    case message::behaviour::MotionCommand::Type::DIRECT_COMMAND: walkDirectly(); return;

                    case message::behaviour::MotionCommand::Type::BALL_APPROACH: visionWalkPath(); return;

                    case message::behaviour::MotionCommand::Type::WALK_TO_STATE: visionWalkPath(); return;

                    case message::behaviour::MotionCommand::Type::ROTATE_ON_SPOT: rotateOnSpot(); return;

                    case message::behaviour::MotionCommand::Type::WALK_TO_READY: walkToReady(); return;
                    // This line should be UNREACHABLE
                    default:
                        log<NUClear::WARN>(
                            fmt::format("Invalid walk path planning command {}.", latestCommand.type.value));
                        emit(std::make_unique<StopCommand>(subsumptionId));
                        return;
                }
            });

        // Save the plan cmd into latestCommand
        on<Trigger<MotionCommand>, Sync<SimpleWalkPathPlanner>>().then(
            [this](const MotionCommand& cmd) { latestCommand = cmd; });
    }

    //-------- Add extra behaviours for path planning here and in the switch case of this file

    void SimpleWalkPathPlanner::walkDirectly() {
        emit(std::move(std::make_unique<WalkCommand>(subsumptionId, latestCommand.walk_command)));
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
    }

    void SimpleWalkPathPlanner::determineSimpleWalkPath(const Ball& ball,
                                                        const Field& field,
                                                        const Sensors& sensors,
                                                        const KickPlan& kickPlan,
                                                        const FieldDescription& fieldDescription) {

        Eigen::Affine3d Htw(sensors.Htw);

        auto now                = NUClear::clock::now();
        float timeSinceBallSeen = std::chrono::duration_cast<std::chrono::nanoseconds>(now - timeBallLastSeen).count()
                                  * (1.0f / std::nano::den);


        Eigen::Vector3d rBWw_temp(ball.position.x(), ball.position.y(), fieldDescription.ball_radius);

        rBWw = timeSinceBallSeen < search_timeout ? rBWw_temp
                                                  : Htw.inverse().linear().leftCols<1>() + Htw.inverse().translation();

        position = (Htw * rBWw).head<2>();

        // TODO Fix Hack Planner:
        float headingChange = 0;
        float sideStep      = 0;
        float speedFactor   = 1;

        if (useLocalisation) {

            // Transform kick target to torso space
            auto fieldPosition = Eigen::Affine2d(field.position);
            Eigen::Affine3d Hfw;
            Hfw.translation() = Eigen::Vector3d(fieldPosition.translation().x(), fieldPosition.translation().y(), 0);
            Hfw.linear() =
                Eigen::AngleAxisd(Eigen::Rotation2Dd(fieldPosition.rotation()).angle(), Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();

            Eigen::Affine3d Htf        = Htw * Hfw.inverse();
            Eigen::Vector3d kickTarget = Htf * Eigen::Vector3d(kickPlan.target.x(), kickPlan.target.y(), 0);

            // approach point:
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

        // If the value of position was less than the slowdown_distance provided in the config file,
        // headingChange is potentially non-zero. This causes the requested angle to turn in the move
        // command at the end of this function to potentially be the in the same direction resultant of the
        // vector between the robot and the ball, as well as the vector from ball to goal (Bryce Tuppurainen
        // - Can someone please confirm or clarify this with my in the PR, this was my understanding when
        // going through this script initially)
        float angle = std::atan2(position.y(), position.x()) + headingChange;

        // If the angle is bound between the maximum provided turning
        // speed use it. Otherwise, the appropriate maximum is used
        angle = std::min(maxTurnSpeed, std::max(angle, -maxTurnSpeed));


        // Euclidean distance to ball (scaleF, scaleF2, scaleS, scaleS2 for Forward and Side
        // respectively, angle is provided above as some float value in radians from the arc-tan, divding
        // this by Pi )
        float scaleF            = 2.0 / (1.0 + std::exp(-a * std::fabs(position.x()) + b)) - 1.0;
        float scaleF2           = angle / M_PI;
        float finalForwardSpeed = speedFactor * forwardSpeed * scaleF * (1.0 - scaleF2);

        float scaleS         = 2.0 / (1.0 + std::exp(-a * std::fabs(position.y()) + b)) - 1.0;
        float scaleS2        = angle / M_PI;
        float finalSideSpeed = -speedFactor * ((0.0 < position.y()) - (position.y() < 0.0)) * sideStep * sideSpeed
                               * scaleS * (1.0 - scaleS2);


        std::unique_ptr<WalkCommand> command =
            std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(finalForwardSpeed, finalSideSpeed, angle));

        emit(std::move(command));
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
    }
    void SimpleWalkPathPlanner::visionWalkPath() {
        log<NUClear::WARN>("walk to ball");
        // Normalize the ball position to obtain the unit vector in torso space
        Eigen::Vector3f unit_vector_to_ball = rBTt / rBTt.norm();
        // Scale the unit vector by forwardSpeed
        Eigen::Vector3f velocity_vector = forwardSpeed * unit_vector_to_ball;
        float angular_velocity          = std::atan2(velocity_vector.y(), velocity_vector.x());
        // Saturate the angular velocity with value maxTurnSpeed
        angular_velocity = std::min(maxTurnSpeed, std::max(angular_velocity, minTurnSpeed));
        log<NUClear::WARN>("angular_velocity", angular_velocity);
        std::unique_ptr<WalkCommand> command =
            std::make_unique<WalkCommand>(subsumptionId,
                                          Eigen::Vector3d(velocity_vector.x(), velocity_vector.y(), angular_velocity));
        emit(std::move(command));
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
    }

    void SimpleWalkPathPlanner::rotateOnSpot() {
        // Get the sign of the last seen ball positionm, such that the robot hopefully rotates towards the ball after
        // it loses it
        // int signOfLastSeenBall = rBTt.y() > 0 ? 1 : -1;
        // log<NUClear::WARN>("rotate on spot : ", signOfLastSeenBall);
        rotateSpeed = rotateSpeed;
        std::unique_ptr<WalkCommand> command =
            std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(rotateSpeedX, rotateSpeedY, rotateSpeed));
        emit(std::move(command));
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
    }

    void SimpleWalkPathPlanner::walkToReady() {
        log<NUClear::WARN>("walk to ready");
        std::unique_ptr<WalkCommand> command =
            std::make_unique<WalkCommand>(subsumptionId,
                                          Eigen::Vector3d(walkToReadySpeedX, walkToReadySpeedY, walkToReadyRotation));
        emit(std::move(command));
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
    }

}  // namespace module::behaviour::planning
