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

#include "HeadController.h"

#include "extension/Configuration.h"

#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/HeadCommand.h"
#include "message/motion/KinematicsModel.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/comparison.h"
#include "utility/math/coordinates.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"


namespace module {
namespace motion {
    using utility::nusight::graph;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;
    using extension::Configuration;
    using message::behaviour::ServoCommand;
    using message::input::Sensors;
    using message::motion::HeadCommand;
    using message::motion::KinematicsModel;
    using utility::behaviour::RegisterAction;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::math::matrix::Transform3D;
    using utility::motion::kinematics::calculateCameraLookJoints;
    using utility::motion::kinematics::calculateHeadJoints;
    using utility::support::Expression;

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
        , updateHandle()
        , lastTime()
        , currentAngles(arma::fill::zeros)
        , goalAngles(arma::fill::zeros) {

        // do a little configurating
        on<Configuration>("HeadController.yaml")
            .then("Head Controller - Configure", [this](const Configuration& config) {
                // Gains
                head_motor_gain   = config["head_motors"]["gain"].as<double>();
                head_motor_torque = config["head_motors"]["torque"].as<double>();

                emit(std::make_unique<HeadCommand>(
                    HeadCommand{config["initial"]["yaw"].as<float>(), config["initial"]["pitch"].as<float>(), false}));

                p_gain = config["p_gain"].as<float>();
            });

        on<Trigger<HeadCommand>>().then("Head Controller - Register Head Command", [this](const HeadCommand& command) {
            goalRobotSpace = command.robotSpace;
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
                emit(graph("HeadController Goal Angles", goalAngles[0], goalAngles[1]));
                // P controller
                currentAngles = p_gain * goalAngles + (1 - p_gain) * currentAngles;

                // Get goal vector from angles
                // Pitch is positive when the robot is looking down by Right hand rule, so negate the pitch
                // The goal angles are for the neck directly, so we have to offset the camera declination again
                arma::vec3 goalHeadUnitVector_world = sphericalToCartesian({1, currentAngles[0], currentAngles[1]});
                // Convert to robot space
                arma::vec3 headUnitVector =
                    goalRobotSpace ? goalHeadUnitVector_world
                                   : Transform3D(convert(sensors.Htw)).rotation() * goalHeadUnitVector_world;
                // Compute inverse kinematics for head
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                // TODO::::MAKE THIS NOT FAIL FOR ANGLES OVER 90deg
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                //!!!!!!!!!!!!!!
                std::vector<std::pair<ServoID, float>> goalAnglesList = calculateHeadJoints(headUnitVector);
                // arma::vec2 goalAngles = cartesianToSpherical(headUnitVector).rows(1,2);

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
                auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                waypoints->reserve(2);
                auto t = NUClear::clock::now();
                for (auto& angle : goalAnglesList) {
                    waypoints->push_back({id,
                                          t,
                                          angle.first,
                                          angle.second,
                                          float(head_motor_gain),
                                          float(head_motor_torque)});  // TODO: support separate gains for each leg
                }
                // Send commands
                emit(std::move(waypoints));
            });

        updateHandle.disable();

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            id,
            "HeadController",
            {std::pair<float, std::set<LimbID>>(30.0, {LimbID::HEAD})},
            [this](const std::set<LimbID>&) {  // Head control gained
                updateHandle.enable();
            },
            [this](const std::set<LimbID>&) {  // Head controll lost
                updateHandle.disable();
            },
            [this](const std::set<ServoID>&) {}  // Servos reached target
        }));
    }

}  // namespace motion
}  // namespace module
