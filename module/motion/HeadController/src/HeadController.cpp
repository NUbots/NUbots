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

#include "message/behaviour/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/motion/HeadCommand.hpp"
#include "message/motion/KinematicsModel.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/comparison.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/motion/InverseKinematics.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::motion {
    using utility::nusight::graph;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;
    using extension::Configuration;
    using message::behaviour::ServoCommands;
    using message::input::Sensors;
    using message::motion::HeadCommand;
    using message::motion::KinematicsModel;
    using utility::behaviour::RegisterAction;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::motion::kinematics::calculateHeadJoints;

    // internal only callback messages to start and stop our action
    struct ExecuteHeadController {};

    HeadController::HeadController(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this))
        , min_yaw(0.0)
        , max_yaw(0.0)
        , min_pitch(0.0)
        , max_pitch(0.0)
        , head_motor_gain(0.0)
        , head_motor_torque(0.0)
        , p_gain(0.0)
        , currentAngles(Eigen::Vector2f::Zero())
        , goalAngles(Eigen::Vector2f::Zero()) {

        // do a little configurating
        on<Configuration>("HeadController.yaml")
            .then("Head Controller - Configure", [this](const Configuration& config) {
                log_level = config["log_level"].as<NUClear::LogLevel>();

                // Gains
                head_motor_gain   = config["head_motors"]["gain"].as<double>();
                head_motor_torque = config["head_motors"]["torque"].as<double>();

                emit(std::make_unique<HeadCommand>(
                    HeadCommand{config["initial"]["yaw"].as<float>(), config["initial"]["pitch"].as<float>(), false}));

                p_gain = config["p_gain"].as<float>();
            });

        on<Trigger<HeadCommand>>().then("Head Controller - Register Head Command", [this](const HeadCommand& command) {
            goalRobotSpace = command.robot_space;
            if (goalRobotSpace) {
                goalAngles = {utility::math::clamp(float(min_yaw), command.yaw, float(max_yaw)),
                              utility::math::clamp(float(min_pitch), command.pitch, float(max_pitch))};
            }
            else {
                goalAngles = {utility::math::clamp(float(min_yaw), command.yaw, float(max_yaw)),
                              -utility::math::clamp(float(min_pitch), command.pitch, float(max_pitch))};
            }
        });

        updateHandle = on<Trigger<Sensors>, With<KinematicsModel>, Single, Priority::HIGH>().then(
            "Head Controller - Update Head Position",
            [this](const Sensors& sensors, const KinematicsModel& kinematicsModel) {
                emit(graph("HeadController Goal Angles", goalAngles.x(), goalAngles.y()));
                // P controller
                currentAngles = p_gain * goalAngles + (1 - p_gain) * currentAngles;

                // Get goal vector from angles
                // Pitch is positive when the robot is looking down by Right hand rule, so negate the pitch
                // The goal angles are for the neck directly, so we have to offset the camera declination again
                Eigen::Vector3f goalHeadUnitVector_world =
                    sphericalToCartesian(Eigen::Vector3f(1, currentAngles.x(), currentAngles.y()));
                // Convert to robot space
                Eigen::Vector3f headUnitVector =
                    goalRobotSpace ? goalHeadUnitVector_world
                                   : Eigen::Affine3d(sensors.Htw).rotation().cast<float>() * goalHeadUnitVector_world;
                // Compute inverse kinematics for head
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                // TODO(MotionTeam): :::MAKE THIS NOT FAIL FOR ANGLES OVER 90deg
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                std::vector<std::pair<ServoID, float>> goalAnglesList = calculateHeadJoints(headUnitVector);
                // Eigen::Vector2f goalAngles = cartesianToSpherical(headUnitVector).tail<2>();

                // head limits
                max_yaw   = kinematicsModel.head.MAX_YAW;
                min_yaw   = kinematicsModel.head.MIN_YAW;
                max_pitch = kinematicsModel.head.MAX_PITCH;
                min_pitch = kinematicsModel.head.MIN_PITCH;

                // Clamp head angles
                float pitch = 0;
                float yaw   = 0;
                for (auto& angle : goalAnglesList) {
                    if (angle.first == ServoID::HEAD_PITCH) {
                        angle.second = std::fmin(std::fmax(angle.second, min_pitch), max_pitch);
                        pitch        = angle.second;
                    }
                    else if (angle.first == ServoID::HEAD_YAW) {
                        angle.second = std::fmin(std::fmax(angle.second, min_yaw), max_yaw);
                        yaw          = angle.second;
                    }
                }

                emit(graph("HeadController Final Angles", yaw, -pitch));
                // log("HeadController Final Angles", yaw, -pitch);


                // Create message
                auto waypoints = std::make_unique<ServoCommands>();
                waypoints->commands.reserve(2);
                auto t = NUClear::clock::now();
                for (auto& angle : goalAnglesList) {
                    waypoints->commands.emplace_back(id,
                                                     t,
                                                     angle.first,
                                                     angle.second,
                                                     float(head_motor_gain),
                                                     float(head_motor_torque));
                }
                // Send commands
                emit(waypoints);
            });

        updateHandle.disable();

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            id,
            "HeadController",
            {std::pair<float, std::set<LimbID>>(30.0, {LimbID::HEAD})},
            [this](const std::set<LimbID>& /* limbs */) {  // Head control gained
                updateHandle.enable();
            },
            [this](const std::set<LimbID>& /* limbs */) {  // Head control lost
                updateHandle.disable();
            },
            [](const std::set<ServoID>& /* servos */) {}  // Servos reached target
        }));
    }
}  // namespace module::motion
