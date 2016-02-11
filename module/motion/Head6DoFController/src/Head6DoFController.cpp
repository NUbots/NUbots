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

#include "message/support/Configuration.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/motion/RobotModels.h"
#include "message/behaviour/ServoCommand.h"
#include "message/input/ServoID.h"
#include "message/input/Sensors.h"
#include "message/behaviour/Action.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace motion {

    using message::support::Configuration;
    using message::behaviour::RegisterAction;
    using message::behaviour::ActionPriorites;
    using message::input::ServoID;
    using message::input::ServoSide;
    using message::input::Sensors;
    using message::input::LimbID;

    using utility::math::matrix::Transform3D;
    using utility::motion::kinematics::DarwinModel;
    using utility::motion::kinematics::Side;

    using utility::support::Expression;
    using message::behaviour::ServoCommand;


    Head6DoFController::Head6DoFController(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)),
    id(size_t(this) * size_t(this) - size_t(this)) 
    {

        on<Configuration>("Head6DoFController.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file Head6DoFController.yaml
            foot_separation = config["foot_separation"].as<Expression>();
			body_angle = config["body_angle"].as<Expression>();
			
			float yaw = config["robot_to_head"]["yaw"].as<Expression>();
			float pitch = config["robot_to_head"]["pitch"].as<Expression>();
			arma::vec3 pos = config["robot_to_head"]["pos"].as<arma::vec>();
            
            robot_to_head_scale = config["robot_to_head"]["scale"].as<Expression>();
			robot_to_head = Transform3D::createTranslation(test_pos) * Transform3D::createRotationZ(test_yaw) * Transform3D::createRotationY(test_pitch);

			l_arm = config["l_arm"].as<arma::vec>(); 
			r_arm = config["r_arm"].as<arma::vec>();

			updatePriority(100);

        });

        on<Every<60,Per<std::chrono::seconds>>, With<Sensors>, With<PresenceUserState>, Single
        >().then([this](const Sensors& sensors
                        const PresenceUserState& user){
			
        	//Record current arm position:
        	arma::vec3 prevArmJointsL = {
        								sensors.servos[int(ServoID::L_SHOULDER_PITCH)].presentPosition,
        								sensors.servos[int(ServoID::L_SHOULDER_ROLL)].presentPosition,
        								sensors.servos[int(ServoID::L_ELBOW)].presentPosition,
        								};
        	arma::vec3 prevArmJointsR = {
        								sensors.servos[int(ServoID::R_SHOULDER_PITCH)].presentPosition,
        								sensors.servos[int(ServoID::R_SHOULDER_ROLL)].presentPosition,
        								sensors.servos[int(ServoID::R_ELBOW)].presentPosition,
        								};

			//Adjust arm position
        	int max_number_of_iterations = 20;

            Transform3D robotCamPose = user.camPose;
            robotCamPose.translation() = robot_to_head_scale * robotCamPose.translation()
			auto joints = utility::motion::kinematics::setHeadPoseFromFeet<DarwinModel>(robotCamPose * robot_to_head, foot_separation, body_angle);
        	
            //TODO: fix arms
        	// auto arm_jointsL = utility::motion::kinematics::setArm<DarwinModel>(l_arm, true, max_number_of_iterations, prevArmJointsL);
        	// auto arm_jointsR = utility::motion::kinematics::setArm<DarwinModel>(r_arm, false, max_number_of_iterations, prevArmJointsR);
        	// joints.insert(joints.end(), arm_jointsL.begin(), arm_jointsL.end());
        	// joints.insert(joints.end(), arm_jointsR.begin(), arm_jointsR.end());


	        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
	        waypoints->reserve(16);

	        NUClear::clock::time_point time = NUClear::clock::now();

	        for (auto& joint : joints) {
	            waypoints->push_back({ id, time, joint.first, joint.second, 30, 100 }); // TODO: support separate gains for each leg
        	}	
        	emit(waypoints);

        	// Transform3D R_shoulder_pitch = sensors.forwardKinematics.at(ServoID::R_SHOULDER_PITCH);
        	// Transform3D R_shoulder_roll = sensors.forwardKinematics.at(ServoID::R_SHOULDER_ROLL);
        	// Transform3D R_arm = sensors.forwardKinematics.at(ServoID::R_ELBOW);
        	// Transform3D L_shoulder_pitch = sensors.forwardKinematics.at(ServoID::L_SHOULDER_PITCH);
        	// Transform3D L_shoulder_roll = sensors.forwardKinematics.at(ServoID::L_SHOULDER_ROLL);
        	// Transform3D L_arm = sensors.forwardKinematics.at(ServoID::L_ELBOW);

        	// arma::vec3 zeros = arma::zeros(3);
        	// arma::vec3 zero_pos = utility::motion::kinematics::calculateArmPosition<DarwinModel>(zeros, true);

        	// std::cout << "New zero pos = \n" << zero_pos << std::endl;
        	// std::cout << "Traditional FK R_shoulder_pitch = \n" << R_shoulder_pitch << std::endl;
        	// std::cout << "Traditional FK R_shoulder_roll = \n" << R_shoulder_roll << std::endl;
        	// std::cout << "Traditional FK R_arm = \n" << R_arm << std::endl;
        	// std::cout << "Traditional FK L_shoulder_pitch = \n" << L_shoulder_pitch << std::endl;
        	// std::cout << "Traditional FK L_shoulder_roll = \n" << L_shoulder_roll << std::endl;
        	// std::cout << "Traditional FK L_arm = \n" << L_arm << std::endl;
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "Head 6DoF Controller",
            { std::pair<float, std::set<LimbID>>(0, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::HEAD, LimbID::RIGHT_ARM, LimbID::LEFT_ARM }) },
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
