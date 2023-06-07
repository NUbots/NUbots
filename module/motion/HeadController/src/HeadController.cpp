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

#include "HeadController.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/behaviour/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/motion/HeadCommand.hpp"

#include "utility/actuation/InverseKinematics.hpp"
#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/comparison.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::motion {
    using utility::nusight::graph;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;
    using extension::Configuration;
    using message::actuation::KinematicsModel;
    using message::behaviour::ServoCommands;
    using message::input::Sensors;
    using message::motion::HeadCommand;
    using utility::actuation::kinematics::calculateHeadJoints;
    using utility::behaviour::RegisterAction;
    using utility::math::coordinates::sphericalToCartesian;

    // internal only callback messages to start and stop our action
    struct ExecuteHeadController {};

    HeadController::HeadController(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {

        // do a little configurating
        on<Configuration>("HeadController.yaml")
            .then("Head Controller - Configure", [this](const Configuration& config) {
                log_level                    = config["log_level"].as<NUClear::LogLevel>();
                cfg.head_controller_priority = config["head_controller_priority"].as<double>();
                cfg.head_motor_gain          = config["head_motors"]["gain"].as<double>();
                cfg.head_motor_torque        = config["head_motors"]["torque"].as<double>();
                cfg.smoothing_factor         = config["smoothing_factor"].as<double>();

                //  Move the head to its config specified initial conditions
                emit(std::make_unique<HeadCommand>(HeadCommand{config["initial"]["yaw"].as<double>(),
                                                               config["initial"]["pitch"].as<double>(),
                                                               false}));
            });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
            RegisterAction{subsumption_id,
                           "HeadController",
                           {std::pair<double, std::set<LimbID>>(cfg.head_controller_priority, {LimbID::HEAD})},
                           [this](const std::set<LimbID>& /* limbs */) { /* Head control gained */ },
                           [this](const std::set<LimbID>& /* limbs */) { /* Head control lost */ },
                           [](const std::set<ServoID>& /* servos */) { /* Servos reached target */ }}));

        on<Trigger<HeadCommand>>().then("Head Controller - Register Head Command", [this](const HeadCommand& command) {
            goal_angles = {command.yaw, command.pitch};
            // If switching from non-smoothed to smoothed angle command, reset the initial goal angle to help locking on
            // to the target
            if (smooth == false && command.smooth == true) {
                current_angles = goal_angles;
            }
            smooth = command.smooth;
        });

        on<Trigger<Sensors>, With<KinematicsModel>, Single, Priority::HIGH>().then(
            "Head Controller - Update Head Position",
            [this](const Sensors& sensors, const KinematicsModel& kinematicsModel) {
                emit(graph("HeadController Goal Angles", goal_angles.x(), goal_angles.y()));

                // If smoothing requested, smooth goal angles with exponential filter
                current_angles =
                    smooth ? (cfg.smoothing_factor * goal_angles + (1 - cfg.smoothing_factor) * current_angles)
                           : goal_angles;

                // Get goal vector from angles
                Eigen::Vector3d goal_head_unit_vector_world =
                    sphericalToCartesian(Eigen::Vector3d(1, current_angles.x(), current_angles.y()));
                // Convert to robot space if requested angle is in world space
                Eigen::Vector3d head_unit_vector =
                    goal_robot_space
                        ? goal_head_unit_vector_world
                        : Eigen::Isometry3d(sensors.Htw).rotation().cast<double>() * goal_head_unit_vector_world;

                // Compute inverse kinematics for head
                // TODO(MotionTeam): MAKE THIS NOT FAIL FOR ANGLES OVER 90deg
                std::vector<std::pair<ServoID, double>> goal_angles_list = calculateHeadJoints(head_unit_vector);

                // Store angles for logging
                double pitch = 0;
                double yaw   = 0;

                // Clamp requested head angles with max/min limits
                double max_yaw   = kinematicsModel.head.MAX_YAW;
                double min_yaw   = kinematicsModel.head.MIN_YAW;
                double max_pitch = kinematicsModel.head.MAX_PITCH;
                double min_pitch = kinematicsModel.head.MIN_PITCH;
                for (auto& angle : goal_angles_list) {
                    if (angle.first == ServoID::HEAD_PITCH) {
                        angle.second = utility::math::clamp(min_pitch, angle.second, max_pitch);
                        pitch        = angle.second;
                    }
                    else if (angle.first == ServoID::HEAD_YAW) {
                        angle.second = utility::math::clamp(min_yaw, angle.second, max_yaw);
                        yaw          = angle.second;
                    }
                }

                emit(graph("HeadController Final Angles", yaw, pitch));

                // Create servo command message
                auto waypoints = std::make_unique<ServoCommands>();
                waypoints->commands.reserve(2);
                auto t = NUClear::clock::now();
                for (auto& angle : goal_angles_list) {
                    waypoints->commands.emplace_back(subsumption_id,
                                                     t,
                                                     angle.first,
                                                     angle.second,
                                                     double(cfg.head_motor_gain),
                                                     double(cfg.head_motor_torque));
                }
                // Send commands
                emit(waypoints);
            });
    }
}  // namespace module::motion
