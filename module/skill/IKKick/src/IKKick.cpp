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

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/Kick.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"

namespace module::skill {

    using extension::Configuration;


    using message::actuation::LeftLegIK;
    using message::actuation::RightLegIK;
    using message::actuation::ServoState;
    using message::input::Sensors;
    using LimbID  = utility::input::LimbID;
    using FrameID = utility::input::FrameID;
    using ServoID = utility::input::ServoID;
    using message::actuation::KinematicsModel;
    using message::actuation::ServoCommand;
    using message::skill::Kick;
    using message::support::FieldDescription;

    IKKick::IKKick(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("IKKick.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            balancer.configure(config);
            kicker.configure(config);

            foot_separation = config["balancer"]["foot_separation"].as<float>();

            cfg.gain_legs = config["servo"]["gain"].as<float>();
            cfg.torque    = config["servo"]["torque"].as<float>();

            auto balance_config = config["active_balance"];
            feedback_active     = balance_config["enabled"].as<bool>();
            feedback_balancer.configure(balance_config);
        });

        on<Start<Kick>, With<Sensors>, With<KinematicsModel>>().then(
            [this](const Kick& kick, const Sensors& sensors, const KinematicsModel& kinematics_model) {
                // 4x4 homogeneous transform matrices for left foot and right foot relative to torso
                Eigen::Isometry3d left_foot(sensors.Htx[FrameID::L_ANKLE_ROLL]);
                Eigen::Isometry3d right_foot(sensors.Htx[FrameID::R_ANKLE_ROLL]);

                // Work out which of our feet are going to be the support foot
                // Store the support foot and kick foot
                if (kick.target.y() < 0) {
                    support_foot = LimbID::LEFT_LEG;
                }
                else {
                    support_foot = LimbID::RIGHT_LEG;
                }

                Eigen::Isometry3d torso_pose =
                    (support_foot == LimbID::LEFT_LEG) ? left_foot.inverse() : right_foot.inverse();

                Eigen::Isometry3d Htg = sensors.Htw * sensors.Hrw.inverse();

                // Put the ball position from vision into torso coordinates by transforming the command target
                Eigen::Vector3d target_torso = Htg * kick.target;

                // Put the ball position into support foot coordinates
                Eigen::Vector3d target_support_foot = torso_pose * target_torso;

                // Put the goal from vision into torso coordinates
                Eigen::Vector3d direction_torso(Htg * kick.direction);

                // Put the goal into support foot coordinates. Note that this transforms direction_torso as a vector,
                // as opposed to transforming it as a point
                Eigen::Vector3d direction_support_foot = torso_pose.rotation() * direction_torso;

                Eigen::Vector3d ball_position = target_support_foot;
                ball_position.z()             = 0.05;  // TODO: get ball height from config
                Eigen::Vector3d goal_position = direction_support_foot;
                goal_position.z()             = 0.0;

                balancer.set_kick_parameters(support_foot, ball_position, goal_position);
                kicker.set_kick_parameters(support_foot, ball_position, goal_position);

                balancer.start(kinematics_model, sensors);
            });

        on<Provide<Kick>,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           With<Sensors>,
           With<KinematicsModel>,
           Single>()
            .then([this](const Sensors& sensors, const KinematicsModel& kinematics_model) {
                // Setup kick variables
                LimbID kick_foot;
                if (support_foot == LimbID::RIGHT_LEG) {
                    kick_foot = LimbID::LEFT_LEG;
                }
                else {
                    kick_foot = LimbID::RIGHT_LEG;
                }

                int negative_if_kick_right = kick_foot == LimbID::RIGHT_LEG ? -1 : 1;

                // State checker
                if (balancer.is_stable()) {
                    kicker.start(kinematics_model, sensors);
                }

                if (kicker.is_stable()) {
                    kicker.stop(sensors);
                    balancer.stop(sensors);
                }

                // Todo: check what needs to be done here; is this the end condition?
                if (balancer.is_finished()) {
                    emit<Task>(std::make_unique<Done>());
                    return;
                }

                // Do things based on current state

                Eigen::Isometry3d kick_foot_goal;
                Eigen::Isometry3d support_foot_goal;

                // Move torso over support foot
                if (balancer.is_running()) {
                    Eigen::Isometry3d support_foot_pose = balancer.get_foot_pose(sensors);
                    support_foot_goal                   = support_foot_pose;
                    kick_foot_goal =
                        support_foot_pose.translate(Eigen::Vector3d(0, negative_if_kick_right * foot_separation, 0));
                }

                // Move foot to ball to kick
                if (kicker.is_running()) {
                    kick_foot_goal = kick_foot_goal * kicker.get_foot_pose(sensors);
                }

                // Balance based on the IMU
                Eigen::Isometry3f support_foot_goal_float(support_foot_goal.cast<float>());
                if (feedback_active) {
                    feedback_balancer.balance(kinematics_model, support_foot_goal_float, support_foot, sensors);
                }
                support_foot_goal = support_foot_goal_float.cast<double>();  // yuk

                // Create IK tasks for the legs
                auto left_leg  = std::make_unique<LeftLegIK>();
                auto right_leg = std::make_unique<RightLegIK>();

                // Run immediately
                left_leg->time  = NUClear::clock::now();
                right_leg->time = NUClear::clock::now();

                // Get position of the feet in the torso frame
                left_leg->Htf  = kick_foot == LimbID::LEFT_LEG ? kick_foot_goal : support_foot_goal;
                right_leg->Htf = kick_foot == LimbID::RIGHT_LEG ? kick_foot_goal : support_foot_goal;

                // Set the servo states (gain and torque) for the legs
                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::LEFT_LEG)) {
                    left_leg->servos[id] = ServoState(cfg.gain_legs, cfg.torque);
                }
                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::RIGHT_LEG)) {
                    right_leg->servos[id] = ServoState(cfg.gain_legs, cfg.torque);
                }

                // Emit the IK tasks
                emit<Task>(std::move(left_leg));
                emit<Task>(std::move(right_leg));
            });
    };

}  // namespace module::skill
