/*
 * MIT License
 *
 * Copyright (c) 2015 NUbots
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

#include "IKKick.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/behaviour/KickPlan.hpp"
#include "message/behaviour/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/motion/KickCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/actuation/InverseKinematics.hpp"
#include "utility/behaviour/Action.hpp"
#include "utility/input/FrameID.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::motion {

    using extension::Configuration;

    using message::input::Sensors;
    using LimbID  = utility::input::LimbID;
    using FrameID = utility::input::FrameID;
    using ServoID = utility::input::ServoID;
    using message::behaviour::KickPlan;
    using message::behaviour::ServoCommands;
    using message::motion::IKKickParams;
    using message::motion::KickCommand;
    using message::motion::KickFinished;
    using message::motion::StopCommand;
    using KickType = message::behaviour::KickPlan::KickType;
    using message::actuation::KinematicsModel;
    using message::support::FieldDescription;

    using utility::actuation::kinematics::calculate_leg_joints;
    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;
    using utility::nusight::graph;

    struct ExecuteKick {};
    struct FinishKick {};

    IKKick::IKKick(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , supportFoot()
        , ballPosition(Eigen::Vector3d::Zero())
        , goalPosition(Eigen::Vector3d::Zero())
        , subsumptionId(size_t(this) * size_t(this) - size_t(this))
        , leftFootIsSupport(false)
        , foot_separation(0.0f)
        , KICK_PRIORITY(0.0f)
        , EXECUTION_PRIORITY(0.0f)
        , feedback_active(false)
        , feedbackBalancer()
        , balancer()
        , kicker()
        , updater() {

        on<Configuration>("IKKick.yaml").then([this](const Configuration& config) {
            balancer.configure(config);
            kicker.configure(config);

            KICK_PRIORITY      = config["kick_priority"].as<float>();
            EXECUTION_PRIORITY = config["execution_priority"].as<float>();

            foot_separation = config["balancer"]["foot_separation"].as<float>();

            gain_legs = config["servo"]["gain"].as<float>();
            torque    = config["servo"]["torque"].as<float>();

            auto& balanceConfig = config["active_balance"];
            feedback_active     = balanceConfig["enabled"].as<bool>();
            feedbackBalancer.configure(balanceConfig);

            // Emit useful info to KickPlanner
            emit(std::make_unique<IKKickParams>(IKKickParams(config["balancer"]["stand_height"].as<float>())));
        });

        on<Startup>().then("IKKick Startup", [this] {
            // Default kick plan at enemy goals
            emit(std::make_unique<KickPlan>(KickPlan(Eigen::Vector2d(4.5, 0), KickPlan::KickType::IK_KICK)));
        });

        on<Trigger<KickCommand>>().then([this] {
            // We want to kick!

            emit(std::make_unique<StopCommand>(subsumptionId));  // Stop the walk

            updatePriority(KICK_PRIORITY);
        });

        on<Trigger<ExecuteKick>, With<KickCommand, Sensors, KinematicsModel>>().then(
            [this](const KickCommand& command, const Sensors& sensors, const KinematicsModel& kinematicsModel) {
                // Enable our kick pather
                updater.enable();
                updatePriority(EXECUTION_PRIORITY);


                // 4x4 homogeneous transform matrices for left foot and right foot relative to torso
                Eigen::Isometry3d leftFoot(sensors.Htx[FrameID::L_ANKLE_ROLL]);
                Eigen::Isometry3d rightFoot(sensors.Htx[FrameID::R_ANKLE_ROLL]);

                // Work out which of our feet are going to be the support foot
                // Store the support foot and kick foot
                if (command.target[1] < 0) {
                    supportFoot = LimbID::LEFT_LEG;
                }
                else {
                    supportFoot = LimbID::RIGHT_LEG;
                }

                Eigen::Isometry3d torsoPose =
                    (supportFoot == LimbID::LEFT_LEG) ? leftFoot.inverse() : rightFoot.inverse();

                Eigen::Isometry3d Htg = Eigen::Isometry3d(sensors.Hgt).inverse();

                // Put the ball position from vision into torso coordinates by transforming the command target
                Eigen::Vector3d targetTorso = Htg * command.target;

                // Put the ball position into support foot coordinates
                Eigen::Vector3d targetSupportFoot = torsoPose * targetTorso;

                // Put the goal from vision into torso coordinates
                Eigen::Vector3d directionTorso(Htg * command.direction);

                // Put the goal into support foot coordinates. Note that this transforms directionTorso as a vector,
                // as opposed to transforming it as a point
                Eigen::Vector3d directionSupportFoot = torsoPose.rotation() * directionTorso;

                Eigen::Vector3d ballPosition = targetSupportFoot;
                ballPosition.z()             = 0.05;  // TODO: get ball height from config
                Eigen::Vector3d goalPosition = directionSupportFoot;
                goalPosition.z()             = 0.0;

                balancer.setKickParameters(supportFoot, ballPosition, goalPosition);
                kicker.setKickParameters(supportFoot, ballPosition, goalPosition);

                balancer.start(kinematicsModel, sensors);
            });

        updater = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors, KinematicsModel>, Single>().then(
            [this](const Sensors& sensors, const KinematicsModel& kinematicsModel) {
                // Setup kick variables
                LimbID kickFoot;
                if (supportFoot == LimbID::RIGHT_LEG) {
                    kickFoot = LimbID::LEFT_LEG;
                }
                else {
                    kickFoot = LimbID::RIGHT_LEG;
                }

                int negativeIfKickRight = kickFoot == LimbID::RIGHT_LEG ? -1 : 1;

                // State checker
                if (balancer.isStable()) {
                    kicker.start(kinematicsModel, sensors);
                }

                if (kicker.isStable()) {
                    kicker.stop(sensors);
                    balancer.stop(sensors);
                }

                if (balancer.isFinished()) {
                    emit(std::move(std::make_unique<FinishKick>()));
                }

                // Do things based on current state

                Eigen::Isometry3d kickFootGoal;
                Eigen::Isometry3d supportFootGoal;

                // Move torso over support foot
                if (balancer.isRunning()) {
                    Eigen::Isometry3d supportFootPose = balancer.getFootPose(sensors);
                    supportFootGoal                   = supportFootPose;
                    kickFootGoal =
                        supportFootPose.translate(Eigen::Vector3d(0, negativeIfKickRight * foot_separation, 0));
                }

                // Move foot to ball to kick
                if (kicker.isRunning()) {
                    kickFootGoal = kickFootGoal * kicker.getFootPose(sensors);
                }

                // Balance based on the IMU
                Eigen::Isometry3f supportFootGoalFloat(supportFootGoal.cast<float>());
                if (feedback_active) {
                    feedbackBalancer.balance(kinematicsModel, supportFootGoalFloat, supportFoot, sensors);
                }
                supportFootGoal = supportFootGoalFloat.cast<double>();  // yuk

                // Calculate IK and send waypoints
                std::vector<std::pair<ServoID, float>> joints;

                // IK
                auto kickJoints    = calculate_leg_joints(kinematicsModel, kickFootGoal, kickFoot);
                auto supportJoints = calculate_leg_joints(kinematicsModel, supportFootGoal, supportFoot);

                // Combine left and right legs
                joints.insert(joints.end(), kickJoints.begin(), kickJoints.end());
                joints.insert(joints.end(), supportJoints.begin(), supportJoints.end());

                // Create message to send to servos
                auto waypoints = std::make_unique<ServoCommands>();
                waypoints->commands.reserve(16);

                // Goal time is by next frame
                NUClear::clock::time_point time = NUClear::clock::now();

                // Push back each servo command
                for (auto& joint : joints) {
                    waypoints->commands.emplace_back(subsumptionId, time, joint.first, joint.second, gain_legs, torque);
                }

                // Send message
                emit(waypoints);
            });

        updater.disable();

        on<Trigger<FinishKick>>().then([this] {
            emit(std::move(std::make_unique<KickFinished>()));
            updater.disable();
            updatePriority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
            RegisterAction{subsumptionId,
                           "IK Kick",
                           {std::pair<float, std::set<LimbID>>(
                               0,
                               {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
                           [this](const std::set<LimbID>&) { emit(std::make_unique<ExecuteKick>()); },
                           [this](const std::set<LimbID>&) { emit(std::make_unique<FinishKick>()); },
                           [this](const std::set<ServoID>&) {}}));
    }

    void IKKick::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {priority}}));
    }
}  // namespace module::motion
