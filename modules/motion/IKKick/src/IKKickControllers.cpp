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

        tolerance = config["balancer"]["tolerance"].as<float>();
	}

	void FootLifter::configure(const Configuration<IKKickConfig>& config){
		motion_gain = config["lifter"]["motion_gain"].as<float>();
        lift_foot_height = config["lifter"]["lift_foot_height"].as<float>();
        lift_foot_back = config["lifter"]["lift_foot_back"].as<float>();
        velocity = config["lifter"]["velocity"].as<float>();
	}

	void Kicker::configure(const Configuration<IKKickConfig>& config){
		motion_gain = config["kicker"]["motion_gain"].as<float>();
	}

    void KickBalancer::computeMotion(const Sensors& sensors){

    }

    void FootLifter::computeMotion(const Sensors& sensors){
        startPose = getTorsoPose(sensors);
        finishPose = startPose.translate(arma::vec3({-lift_foot_back,0,lift_foot_height}));
        distance = arma::norm(startPose.translation() - finishPose.translation());
        motionStartTime = sensors.timestamp;
    }

    void Kicker::computeMotion(const Sensors& sensors){

    }


	Transform3D KickBalancer::getFootPose(const Sensors& sensors, float deltaT){
		    
        // Obtain the position of the torso and the direction in which the torso needs to move
        // The position that the COM needs to move to in support foot coordinates
        int negativeIfRight = supportFoot == LimbID::LEFT_LEG ? 1 : -1;
        Transform3D torsoTarget = arma::eye(4,4);
        torsoTarget.submat(0,3,3,3) = arma::vec({0, negativeIfRight * DarwinModel::Leg::FOOT_CENTRE_TO_ANKLE_CENTRE, stand_height,1}); 

        Transform3D torsoPose = getTorsoPose(sensors);
        torsoPose.submat(0,0,2,2) = arma::eye(3,3);

        //WARNING: DO NOT SWAP STABLE CHECK AND newTorsoPose OR YOU WILL BREAK ROBOTS
        stable = (arma::norm(torsoPose.submat(0,3,2,3) - torsoTarget.submat(0,3,2,3)) < tolerance);
        // std::cout << "stable = " << stable << std::endl;
        
        Transform3D newTorsoPose = utility::math::matrix::Transform3D::interpolate(torsoPose, torsoTarget, deltaT * motion_gain);
        // std::cout << "torsoPose = \n" << torsoPose << std::endl;
        // std::cout << "torsoTarget = \n" << torsoTarget << std::endl;
        // std::cout << "newTorsoPose = \n" << newTorsoPose << std::endl;

        return newTorsoPose.i();
    }

    Transform3D FootLifter::getFootPose(const Sensors& sensors, float deltaT){
        double elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(sensors.timestamp - motionStartTime).count() * 1e-6;
        float alpha = std::fmax(0,std::fmin(velocity * elapsedTime / distance,1));
        // std::cout << "alpha = " << alpha << std::endl;
		return utility::math::matrix::Transform3D::interpolate(startPose,finishPose,alpha).i();
	}

	Transform3D Kicker::getFootPose(const Sensors& sensors, float deltaT){
        Transform3D goal;
        goal = goal.translate(arma::vec3({0.05,0,0}));
		return goal;
	}
}
}