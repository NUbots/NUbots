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
	    stand_height = config["balancer"]["stand_height"].as<float>();
        forward_lean = config["balancer"]["forward_lean"].as<float>();
        foot_separation = config["balancer"]["foot_separation"].as<float>();

        forward_velocity = config["balancer"]["forward_velocity"].as<float>();
        return_velocity = config["balancer"]["return_velocity"].as<float>();
	}

	void FootLifter::configure(const Configuration<IKKickConfig>& config){
        lift_foot_height = config["lifter"]["lift_foot_height"].as<float>();
        lift_foot_back = config["lifter"]["lift_foot_back"].as<float>();
        
        forward_velocity = config["lifter"]["forward_velocity"].as<float>();
        return_velocity = config["lifter"]["return_velocity"].as<float>();
	}

	void Kicker::configure(const Configuration<IKKickConfig>& config){
        forward_velocity = config["kicker"]["forward_velocity"].as<float>();
        return_velocity = config["kicker"]["return_velocity"].as<float>();
	}

    void KickBalancer::computeMotion(const Sensors& sensors){
        Transform3D torsoToFoot = getTorsoPose(sensors);
        
        int negativeIfRight = (supportFoot == LimbID::RIGHT_LEG) ? -1 : 1;
        finishPose = torsoToFoot;
        finishPose.translation() = arma::vec3({forward_lean, negativeIfRight * DarwinModel::Leg::FOOT_CENTRE_TO_ANKLE_CENTRE, stand_height});

        startPose = torsoToFoot.i();
        finishPose = finishPose.i();
        distance = arma::norm(startPose.translation() - finishPose.translation());
    }

    void FootLifter::computeMotion(const Sensors&){
        startPose = arma::eye(4,4);
        
        finishPose = startPose.translate(arma::vec3({-lift_foot_back,0,lift_foot_height}));
        
        distance = arma::norm(startPose.translation() - finishPose.translation());
    }

    void Kicker::computeMotion(const Sensors& sensors){
        startPose = arma::eye(4,4);
        
        Transform3D currentTorso = getTorsoPose(sensors);
        Transform3D currentKickFoot = supportFoot == LimbID::LEFT_LEG ? sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second 
                                                                      : sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
        Transform3D supportToKickFoot = currentKickFoot.i() * currentTorso.i();
        arma::vec3 ballFromKickFoot = supportToKickFoot.transformPoint(ballPosition);
        finishPose = startPose.translate(ballFromKickFoot);

        distance = arma::norm(startPose.translation() - finishPose.translation());
    }
}
}