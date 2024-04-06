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
#include "SplineKick.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/skill/ControlFoot.hpp"
#include "message/skill/Kick.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::actuation::LeftArm;
    using message::actuation::RightArm;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::skill::ControlLeftFoot;
    using message::skill::ControlRightFoot;
    using message::skill::Kick;

    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::nusight::graph;
    using utility::support::Expression;

    SplineKick::SplineKick(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("SplineKick.yaml").then([this](const Configuration& config) {
            // Use configuration here from file SplineKick.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Add kick motion waypoints
            for (const auto& foot_waypoint : config["foot_waypoints"].config) {
                Waypoint<double> waypoint;
                Eigen::Vector4d frame = foot_waypoint.as<Expression>();
                waypoint.time_point   = frame(3);
                waypoint.position     = frame.head<3>();
                kick_generator.add_foot_waypoint(waypoint);
            }
            for (const auto& torso_waypoint : config["torso_waypoints"].config) {
                Waypoint<double> waypoint;
                Eigen::Vector4d frame = torso_waypoint.as<Expression>();
                waypoint.time_point   = frame(3);
                waypoint.position     = frame.head<3>();
                waypoint.orientation  = Eigen::Vector3d(0, config["torso_pitch"].as<Expression>(), 0);
                kick_generator.add_torso_waypoint(waypoint);
            }

            // Reset the kick engine and last update time
            kick_generator.reset();
            last_update_time = NUClear::clock::now();

            // Configure the arms
            for (auto id : utility::input::LimbID::servos_for_arms()) {
                cfg.servo_states[id] = ServoState(config["gains"]["arms"].as<double>(), 100);
            }
            cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_PITCH,
                                           config["arms"]["right_shoulder_pitch"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_PITCH,
                                           config["arms"]["left_shoulder_pitch"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_ROLL,
                                           config["arms"]["right_shoulder_roll"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_ROLL, config["arms"]["left_shoulder_roll"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::R_ELBOW, config["arms"]["right_elbow"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::L_ELBOW, config["arms"]["left_elbow"].as<double>());
        });

        // Start - Runs every time the Kick provider starts (wasn't running)
        on<Start<Kick>>().then([this]() {
            // Reset the last update time and walk engine
            last_update_time = NUClear::clock::now();
            kick_generator.reset();
        });

        on<Provide<Kick>, Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single>().then(
            [this](const Kick& kick, const RunInfo& info) {
                // Compute time since the last update
                auto time_delta =
                    std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - last_update_time)
                        .count();
                last_update_time = NUClear::clock::now();

                // Update the kick engine
                kick_generator.update(time_delta, kick.leg);

                // If this is not a new task and time has elapsed, then we are done kicking.
                if ((info.run_reason != RunInfo::RunReason::NEW_TASK)
                    && kick_generator.get_time() == kick_generator.get_duration()) {
                    emit<Task>(std::make_unique<Done>());
                    return;
                }

                // If this is a new task and time has elapsed, then we need to start a new kick
                if ((info.run_reason == RunInfo::RunReason::NEW_TASK)
                    && kick_generator.get_time() == kick_generator.get_duration()) {
                    // Start a new kick
                    kick_generator.reset();
                }

                // Compute the goal position time
                const NUClear::clock::time_point goal_time =
                    NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);

                // Get desired feet poses in the torso {t} frame from the walk engine
                Eigen::Isometry3d Htl = kick_generator.get_foot_pose(LimbID::LEFT_LEG);
                Eigen::Isometry3d Htr = kick_generator.get_foot_pose(LimbID::RIGHT_LEG);

                // Construct ControlFoot tasks
                emit<Task>(std::make_unique<ControlLeftFoot>(Htl, goal_time, kick_generator.is_left_foot_planted()));
                emit<Task>(std::make_unique<ControlRightFoot>(Htr, goal_time, !kick_generator.is_left_foot_planted()));

                // Construct Arm IK tasks
                auto left_arm  = std::make_unique<LeftArm>();
                auto right_arm = std::make_unique<RightArm>();
                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::RIGHT_ARM)) {
                    right_arm->servos[id] =
                        ServoCommand(goal_time, cfg.arm_positions[ServoID(id)].second, cfg.servo_states[ServoID(id)]);
                }
                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::LEFT_ARM)) {
                    left_arm->servos[id] =
                        ServoCommand(goal_time, cfg.arm_positions[ServoID(id)].second, cfg.servo_states[ServoID(id)]);
                }
                emit<Task>(left_arm, 0, true, "Walk left arm");
                emit<Task>(right_arm, 0, true, "Walk right arm");
            });
    }

}  // namespace module::skill
