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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "HeadController.h"

#include "message/input/ServoID.h"
#include "message/behaviour/Action.h"
#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/support/Configuration.h"
#include "message/motion/HeadCommand.h"
#include "utility/math/coordinates.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/support/yaml_expression.h"
#include "utility/nubugger/NUhelpers.h"


namespace module {
    namespace motion {

        using utility::nubugger::graph;
        using message::input::ServoID;
        using message::input::Sensors;
        using message::behaviour::RegisterAction;
        using message::input::LimbID;
        using message::support::Configuration;
        using message::behaviour::ServoCommand;
        using message::motion::HeadCommand;
        using utility::math::coordinates::sphericalToCartesian;
        using utility::math::coordinates::cartesianToSpherical;
        using utility::motion::kinematics::calculateHeadJoints;
        using utility::motion::kinematics::DarwinModel;
        using utility::support::Expression;

        //internal only callback messages to start and stop our action
        struct ExecuteHeadController {};

        HeadController::HeadController(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

            currentAngles = {0,0};//TODO: set this to current motor positions
            goalAngles = {0,0};//TODO: set this to current motor positions
            //do a little configurating
            on<Configuration>("HeadController.yaml").then("Head Controller - Config", [this] (const Configuration& config) {
                //Gains
                head_motor_gain = config["head_motors"]["gain"].as<double>();
                head_motor_torque = config["head_motors"]["torque"].as<double>();

                //head limits
                max_yaw = DarwinModel::Head::MAX_YAW;
                min_yaw = DarwinModel::Head::MIN_YAW;
                max_pitch = DarwinModel::Head::MAX_PITCH;
                min_pitch = DarwinModel::Head::MIN_PITCH;

                emit(std::make_unique<HeadCommand>( HeadCommand {config["initial"]["yaw"].as<float>(),
                                                                 config["initial"]["pitch"].as<float>(), false}));

                p_gain = config["p_gain"].as<float>();

            });

            on<Trigger<HeadCommand>>().then("Head Controller - Register Head Command", [this](const HeadCommand& command){
                goalRobotSpace = command.robotSpace;
                if(goalRobotSpace) {
                    goalAngles = {command.yaw, command.pitch};
                } else {
                    goalAngles = {command.yaw, -command.pitch};
                }
            });

            updateHandle = on<Trigger<Sensors>, Single, Priority::HIGH>().then("Head Controller - Update Head Position",[this] (const Sensors& sensors) {
                emit(graph("HeadController Goal Angles", goalAngles[0], goalAngles[1]));
                
                //P controller
                currentAngles = p_gain * goalAngles + (1 - p_gain) * currentAngles;

                //Get goal vector from angles
                //Pitch is positive when the robot is looking down by Right hand rule, so negate the pitch
                arma::vec3 goalHeadUnitVector_world = sphericalToCartesian({1, currentAngles[0], currentAngles[1]});
                //Convert to robot space
                arma::vec3 headUnitVector = goalRobotSpace ? goalHeadUnitVector_world : sensors.orientation * goalHeadUnitVector_world;
                //Compute inverse kinematics for head
                std::vector< std::pair<message::input::ServoID, float> > goalAnglesList = calculateHeadJoints<DarwinModel>(headUnitVector);
                // arma::vec2 goalAngles = cartesianToSpherical(headUnitVector).rows(1,2);

                //Clamp head angles
                float pitch = 0;
                float yaw = 0;
                for(auto& angle : goalAnglesList){
                    if(angle.first == ServoID::HEAD_PITCH){
                        angle.second = std::fmin(std::fmax(angle.second, min_pitch), max_pitch);
                        pitch = angle.second;
                    } else if(angle.first == ServoID::HEAD_YAW){
                        angle.second = std::fmin(std::fmax(angle.second, min_yaw), max_yaw);
                        yaw = angle.second;
                    }
                }

                emit(graph("HeadController Final Angles", yaw, -pitch));


                //Create message
                auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                waypoints->reserve(2);
                auto t = NUClear::clock::now();
                for (auto& angle : goalAnglesList) {
                    waypoints->push_back({ id, t, angle.first, angle.second, float(head_motor_gain), float(head_motor_torque) }); // TODO: support separate gains for each leg
                }
                //Send commands
                emit(std::move(waypoints));
            });

            updateHandle.disable();

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
                id,
                "HeadController",
                { std::pair<float, std::set<LimbID>>(30.0 , { LimbID::HEAD }) },
                [this] (const std::set<LimbID>&) { //Head control gained
                    updateHandle.enable();
                },
                [this] (const std::set<LimbID>&) { //Head controll lost
                    updateHandle.disable();
                },
                [this] (const std::set<ServoID>& ) { } //Servos reached target
            }));

        }

    }  // motion
}  // modules
