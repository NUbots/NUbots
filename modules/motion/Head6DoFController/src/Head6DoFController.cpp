/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "Head6DoFController.h"

#include "messages/support/Configuration.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/motion/RobotModels.h"
#include "messages/behaviour/ServoCommand.h"
#include "messages/input/ServoID.h"
#include "messages/behaviour/Action.h"

namespace modules {
namespace motion {

    using messages::support::Configuration;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;
    using messages::input::ServoID;
    using messages::input::LimbID;

    using utility::math::matrix::Transform3D;
    using utility::motion::kinematics::DarwinModel;

    using messages::behaviour::ServoCommand;


    Head6DoFController::Head6DoFController(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)),
    id(size_t(this) * size_t(this) - size_t(this)) 
    {

        on<Configuration>("Head6DoFController.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file Head6DoFController.yaml
            foot_separation = config["foot_separation"].as<float>();
			body_angle = config["body_angle"].as<float>();
			
			float test_yaw = config["test_yaw"].as<float>();
			float test_pitch = config["test_pitch"].as<float>();
			arma::vec3 test_pos = config["test_pos"].as<arma::vec>();

			testHeadPose = Transform3D::createTranslation(test_pos) * Transform3D::createRotationZ(test_yaw) * Transform3D::createRotationY(test_pitch);

			updatePriority(100);
        });

        on<Every<75,Per<std::chrono::seconds>>>().then([this]{
        	auto joints = utility::motion::kinematics::setHeadPoseFromFeet<DarwinModel>(testHeadPose, foot_separation, body_angle);

	        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
	        waypoints->reserve(16);

	        NUClear::clock::time_point time = NUClear::clock::now();

	        for (auto& joint : joints) {
	            waypoints->push_back({ id, time, joint.first, joint.second, 30, 100 }); // TODO: support separate gains for each leg
        	}	
        	emit(waypoints);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "Head 6DoF Controller",
            { std::pair<float, std::set<LimbID>>(0, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::HEAD }) },
            [this] (const std::set<LimbID>&) {
                // emit(std::make_unique<ExecuteGetup>());
            },
            [this] (const std::set<LimbID>&) {
                // emit(std::make_unique<KillGetup>());
            },
            [this] (const std::set<ServoID>& servoSet) {
                //HACK 2014 Jake Fountain, Trent Houliston
                //TODO track set limbs and wait for all to finish
                // if(servoSet.find(ServoID::L_ANKLE_PITCH) != servoSet.end() ||
                //    servoSet.find(ServoID::R_ANKLE_PITCH) != servoSet.end()) {
                //     emit(std::make_unique<KillGetup>());
                // }
            }
        }));
    }


    void Head6DoFController::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { priority }}));
    }
}
}
