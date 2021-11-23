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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#include "MovementController.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/behaviour/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/motion/FootTarget.hpp"
#include "message/motion/KinematicsModel.hpp"
#include "message/motion/TorsoTarget.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/motion/ForwardKinematics.hpp"
#include "utility/motion/InverseKinematics.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module {
    namespace motion {
        namespace walk {

            using extension::Configuration;
            using message::behaviour::ServoCommands;
            using message::input::Sensors;
            using message::motion::FootTarget;
            using message::motion::KinematicsModel;
            using message::motion::TorsoTarget;
            using utility::input::LimbID;
            using utility::input::ServoID;
            using utility::motion::kinematics::calculateGroundSpace;
            using utility::motion::kinematics::calculateLegJoints;
            using utility::nusight::graph;
            using utility::support::Expression;


            MovementController::MovementController(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)) {

                on<Configuration>("MovementController.yaml").then([this](const Configuration& cfg) {
                    // Use configuration here from file MovementController.yaml

                    // Foot controller config
                    foot_controller.config.step_height    = cfg["foot"]["step_height"].as<Expression>();
                    foot_controller.config.scaling_factor = cfg["foot"]["scaling_factor"].as<Expression>();
                    foot_controller.config.integral_steps = cfg["foot"]["integral_steps"].as<Expression>();

                    foot_controller.config.translation_threshold =
                        cfg["foot"]["translation_threshold"].as<Expression>();
                    foot_controller.config.rotation_threshold = cfg["foot"]["rotation_threshold"].as<Expression>();
                    foot_controller.config.max_translation    = cfg["foot"]["max_translation"].as<Expression>();
                    foot_controller.config.max_rotation       = cfg["foot"]["max_rotation"].as<Expression>();

                    // const auto& h = foot_controller.config.step_height;
                    // const auto& s = foot_controller.config.step_steep;
                    // const auto& w = foot_controller.config.well_width;

                    // Constant for f_x and f_z
                    // foot_controller.config.c =
                    //     (std::pow(s, 2 / s) * std::pow(h, 1 / s) * std::pow(s * h + (s * s * h), -1 / s)) / w;

                    // Torso controller config
                    torso_controller.config.translation_threshold =
                        cfg["torso"]["translation_threshold"].as<Expression>();
                    torso_controller.config.rotation_threshold = cfg["torso"]["rotation_threshold"].as<Expression>();
                    torso_controller.config.max_translation    = cfg["torso"]["max_translation"].as<Expression>();
                    torso_controller.config.max_rotation       = cfg["torso"]["max_rotation"].as<Expression>();

                    // Motion controller config
                    config.time_horizon = cfg["time_horizon"].as<Expression>();

                    config.support_gain    = cfg["gains"]["support_gain"].as<Expression>();
                    config.swing_gain      = cfg["gains"]["swing_gain"].as<Expression>();
                    config.swing_lean_gain = cfg["gains"]["swing_lean_gain"].as<Expression>();
                });

                on<Trigger<Sensors>, With<KinematicsModel>, With<FootTarget>, With<TorsoTarget>>().then(
                    [this](const Sensors& sensors,
                           const KinematicsModel& model,
                           const FootTarget& foot_target,
                           const TorsoTarget& torso_target) {
                        using namespace std::chrono;

                        // Set the support and swing matrices
                        Eigen::Affine3d Hts = torso_target.support_foot == TorsoTarget::SupportFoot::LEFT
                                                  ? Eigen::Affine3d(sensors.Htx[ServoID::L_ANKLE_ROLL])
                                                  : Eigen::Affine3d(sensors.Htx[ServoID::R_ANKLE_ROLL]);
                        Eigen::Affine3d Htw = torso_target.support_foot == TorsoTarget::SupportFoot::LEFT
                                                  ? Eigen::Affine3d(sensors.Htx[ServoID::R_ANKLE_ROLL])
                                                  : Eigen::Affine3d(sensors.Htx[ServoID::L_ANKLE_ROLL]);

                        // Set the time now so the calculations are consistent across the methods
                        const auto now       = NUClear::clock::now();
                        auto torso_time_left = duration_cast<duration<double>>(torso_target.timestamp - now).count();
                        auto swing_time_left = duration_cast<duration<double>>(foot_target.timestamp - now).count();

                        // Retrieve the target matrices
                        Eigen::Affine3d Ht_tg(torso_target.Ht_tg);
                        Eigen::Affine3d Hw_tg(foot_target.Hw_tg);

                        // Check if the point to step to is further away than the length of the foot
                        double step_length = Hw_tg.translation().norm();
                        double hip_offset =
                            Eigen::Vector3d(model.leg.HIP_OFFSET_X,
                                            (foot_target.swing_foot == FootTarget::SwingFoot::LEFT ? 1 : -1)
                                                * model.leg.HIP_OFFSET_Y,
                                            model.leg.HIP_OFFSET_Z)
                                .norm();
                        double max_step_length = hip_offset + model.leg.UPPER_LEG_LENGTH + model.leg.LOWER_LEG_LENGTH
                                                 + model.leg.FOOT_HEIGHT;

                        if (step_length > max_step_length) {
                            log<NUClear::WARN>("Swing foot target out of bounds.");
                        }

                        // Retrieve world matrix
                        Eigen::Affine3d Htworld(sensors.Htw);

                        // If we are within time_horizon of our target time, we always make time_horizon our target time
                        // This slows the approach of the foot to the target as it gets closer, for a smoother landing
                        torso_time_left = torso_time_left < config.time_horizon ? config.time_horizon : torso_time_left;
                        swing_time_left = swing_time_left < config.time_horizon ? config.time_horizon : swing_time_left;

                        // Get the gain of the servo from config based on if the foot is to be lifted off the ground
                        double w_gain =
                            foot_target.mode == FootTarget::Mode::STEP ? config.swing_gain : config.swing_lean_gain;

                        // Get ground space
                        Eigen::Affine3d Htg(calculateGroundSpace(Hts, Htworld.inverse()));

                        // Calculate the next torso and next swing foot positions to be targeted
                        Eigen::Affine3d Ht_ng =
                            torso_controller.next_torso(config.time_horizon, torso_time_left, Htg, Ht_tg);
                        Eigen::Affine3d Hw_ng = foot_target.mode == FootTarget::Mode::STEP
                                                    ? foot_controller.next_swing(config.time_horizon,
                                                                                 swing_time_left,
                                                                                 Htw.inverse() * Htg,
                                                                                 Hw_tg)
                                                    : torso_controller.next_torso(config.time_horizon,
                                                                                  swing_time_left,
                                                                                  Htw.inverse() * Htg,
                                                                                  Hw_tg);

                        // Perform IK for the support and swing feet based on the target torso position
                        // w_n: swing foot next target position
                        // t_n: torso next target position
                        Eigen::Affine3d Ht_nw_n = Ht_ng * Hw_ng.inverse();

                        // Get the target homogeneous transformations for each foot
                        // By using g here, we are assuming the support foot is flat on the ground,
                        // and if it's not it'll try to make it flat on the ground
                        const Eigen::Affine3d left_foot =
                            torso_target.support_foot == TorsoTarget::SupportFoot::RIGHT ? Ht_nw_n : Ht_ng;
                        const Eigen::Affine3d right_foot =
                            torso_target.support_foot == TorsoTarget::SupportFoot::RIGHT ? Ht_ng : Ht_nw_n;

                        // Get joints based on the matrices from IK
                        auto left_joints  = calculateLegJoints(model, left_foot, LimbID::LEFT_LEG);
                        auto right_joints = calculateLegJoints(model, right_foot, LimbID::RIGHT_LEG);

                        // The time the servos will arrive at the target position at
                        // Always projecting into the future (time_horizon)
                        // This is to improve smoothness so that the servos are still moving even if there is the
                        // occasional error in servo communication
                        auto projected_time =
                            time_point_cast<NUClear::clock::duration>(now + duration<double>(config.time_horizon));

                        // Create the servo command for each servo, with the join angles from IK, gains from config and
                        // time calculated from time_horizon
                        auto waypoints = std::make_unique<ServoCommands>();

                        for (const auto& joint : left_joints) {
                            waypoints->commands.emplace_back(
                                foot_target.subsumption_id,
                                projected_time,
                                joint.first,
                                joint.second,
                                torso_target.support_foot == TorsoTarget::SupportFoot::RIGHT ? w_gain
                                                                                             : config.support_gain,
                                100);
                        }

                        for (const auto& joint : right_joints) {
                            waypoints->commands.emplace_back(
                                foot_target.subsumption_id,
                                projected_time,
                                joint.first,
                                joint.second,
                                torso_target.support_foot == TorsoTarget::SupportFoot::RIGHT ? config.support_gain
                                                                                             : w_gain,
                                100);
                        }

                        // Emit the servo commands
                        emit(std::move(waypoints));
                    });
            }
        }  // namespace walk
    }      // namespace motion
}  // namespace module
