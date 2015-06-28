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

using messages::input::Sensors;
using messages::input::LimbID;
using messages::input::ServoID;
using utility::math::matrix::Transform3D;
using messages::support::Configuration;

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

            
            // Obtain the position of the torso and the direction in which the torso needs to move
            // The position that the COM needs to move to in support foot coordinates
            auto torsoTarget = arma::vec({0, 0, stand_height}); 

            // Find position vector from support foot to torso in support foot coordinates.
            auto torsoPosition = leftFoot.i().translation();
           


		return leftFootNewPose;
	}

	Transform3D FootLifter::getFootPose(const Sensors& sensors, float deltaT){
		return Transform3D();
	}

	Transform3D Kicker::getFootPose(const Sensors& sensors, float deltaT){
		return Transform3D();
	}
}
}