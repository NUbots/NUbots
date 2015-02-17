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

#include "messages/input/ServoID.h"
#include "messages/behaviour/Action.h"
#include "messages/behaviour/ServoCommand.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"
#include "messages/motion/HeadCommand/h"
#include "utility/math/coordinates.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/support/yaml_armadillo.h"

namespace modules {
    namespace motion {

            using messages::input::ServoID;
            using messages::input::Sensors;
            using messages::behaviour::RegisterAction;
            using messages::input::LimbID;
            using messages::support::Configuration;
            using messages::behaviour::ServoCommand;
            using messages::motion::HeadCommand;
            using utility::math::coordinates::sphericalToCartesian;
            using utility::motion::kinematics::calculateHeadJoints;
            using utility::motion::kinematics::DarwinModel;

            //internal only callback messages to start and stop our action
            struct ExecuteHeadController {};

            HeadController::HeadController(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

                //do a little configurating
                on<Trigger<Configuration<HeadController>>>([this] (const Configuration<HeadController>& config)
                {
                    //Gains                    
                    head_gain = config["head_gain"].as<double>();
                    head_torque = config["head_torque"].as<double>();

                    //head limits
                    min_yaw = config["min_yaw"].as<double>();
                    max_yaw = config["max_yaw"].as<double>();
                    min_pitch = config["min_pitch"].as<double>();
                    max_pitch = config["max_pitch"].as<double>();
                });

                on<Trigger<Sensors>, With<HeadCommand>>([this] (const Sensors& sensors, const HeadCommand& command) {

                    //Get goal vector from angles
                    arma::vec3 goalHeadUnitVector_world = sphericalToCartesian({1,command.yaw,command.pitch});
                    //Convert to robot space
                    arma::vec3 headUnitVector =  sensors.orientation * goalHeadUnitVector_world;
                    //Compute inverse kinematics for head
                    std::vector< std::pair<messages::input::ServoID, float> > goalAngles = calculateHeadJoints<DarwinModel>(headUnitVector);

                    //Clamp head angles
                    goalAngles[ServoID::HEAD_PITCH].second = arma::clamp(goalAngles[ServoID::HEAD_PITCH].second, min_pitch, max_pitch);
                    goalAngles[ServoID::HEAD_YAW].second = arma::clamp(goalAngles[ServoID::HEAD_YAW].second, min_yaw, max_yaw);

                    //Create message
                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                    waypoints->reserve(2);
                    auto t = NUClear::clock::now();
                    for (auto& angle : goalAngles) {
                        waypoints->push_back({ id, t, angle.first, angle.second, float(head_gain), float(head_torque) }); // TODO: support separate gains for each leg
                    }
                    //Send commands
                    emit(std::move(waypoints));
                });



                on<Trigger<ExecuteHeadController>>([this] (const ExecuteHeadController&) {                

                });

                emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
                    id,
                    "HeadController",
                    { std::pair<float, std::set<LimbID>>(30.0, { LimbID::HEAD }) },
                    [this] (const std::set<LimbID>&) {
                        emit(std::make_unique<ExecuteHeadController>());
                    },
                    [this] (const std::set<LimbID>&) { },
                    [this] (const std::set<ServoID>&) { }
                }));
            }

    }  // motion
}  // modules
