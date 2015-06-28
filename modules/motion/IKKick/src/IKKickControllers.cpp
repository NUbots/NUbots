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

#include "IKKickControllers.h"
#include "utility/motion/RobotModels.h"

using messages::input::Sensors;
using messages::input::LimbID;
using messages::input::ServoID;
using messages::support::Configuration;
using utility::math::matrix::Transform3D;
using utility::motion::kinematics::DarwinModel;

namespace modules{
namespace motion{

	void KickBalancer::configure(const Configuration<IKKickConfig>& config){
		motion_gain = config["balancer"]["motion_gain"].as<float>();
	    stand_height = config["balancer"]["stand_height"].as<float>();
	}

	void FootLifter::configure(const Configuration<IKKickConfig>& config){
		motion_gain = config["lifter"]["motion_gain"].as<float>();
        lift_foot_height = config["lifter"]["lift_foot_height"].as<float>();
        lift_foot_back = config["lifter"]["lift_foot_back"].as<float>();
	}

	void Kicker::configure(const Configuration<IKKickConfig>& config){
		motion_gain = config["kicker"]["motion_gain"].as<float>();
	}

	Transform3D KickBalancer::getFootPose(const Sensors& sensors, float deltaT){
		    
        // Get our foot positions
        Transform3D leftFoot = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
        Transform3D rightFoot = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

        int negativeIfRight = supportFoot == LimbID::LEFT_LEG ? 1 : -1;

        // Obtain the position of the torso and the direction in which the torso needs to move
        // The position that the COM needs to move to in support foot coordinates
        Transform3D torsoTarget = arma::eye(4,4);
        torsoTarget.submat(0,3,3,3) = arma::vec({0, negativeIfRight * DarwinModel::Leg::FOOT_CENTRE_TO_ANKLE_CENTRE, stand_height,1}); 

        // Find position vector from support foot to torso in support foot coordinates.
        Transform3D torsoPose = supportFoot == LimbID::LEFT_LEG ? leftFoot.i() : rightFoot.i();
        std::cout << "torsoPose = \n" << torsoPose << std::endl;

        Transform3D newSupportFootPose = utility::math::matrix::Transform3D::interpolate(torsoPose, torsoTarget, deltaT * motion_gain).i();

		return torsoTarget.i();
	}

	Transform3D FootLifter::getFootPose(const Sensors& sensors, float deltaT){
		return Transform3D();
	}

	Transform3D Kicker::getFootPose(const Sensors& sensors, float deltaT){
		return Transform3D();
	}
}
}