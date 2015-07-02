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
        foot_separation = config["balancer"]["foot_separation"].as<float>();
	}

	void FootLifter::configure(const Configuration<IKKickConfig>& config){
		// motion_gain = config["lifter"]["motion_gain"].as<float>();
        lift_foot_height = config["lifter"]["lift_foot_height"].as<float>();
        lift_foot_back = config["lifter"]["lift_foot_back"].as<float>();
        velocity = config["lifter"]["velocity"].as<float>();
	}

	void Kicker::configure(const Configuration<IKKickConfig>& config){
		// motion_gain = config["kicker"]["motion_gain"].as<float>();
        kick_velocity = config["kicker"]["kick_velocity"].as<float>();
        return_velocity = config["kicker"]["return_velocity"].as<float>();
	}

    void KickBalancer::computeMotion(const Sensors& sensors){
        torsoOrientation = getTorsoPose(sensors);
        centreOfMass_foot = torsoOrientation.transformPoint(sensors.centreOfMass.rows(0,2));
       // torsoOrientation.translation() = arma::zeros(3);
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


	Transform3D KickBalancer::getFootPose(const Sensors& sensors, float deltaT){
		    
        // Obtain the position of the torso and the direction in which the torso needs to move
        // The position that the COM needs to move to in support foot coordinates
        int negativeIfRight = supportFoot == LimbID::LEFT_LEG ? 1 : -1;

        Transform3D torsoPose = getTorsoPose(sensors);
        
        // centreOfMass_foot = torsoPose.transformPoint(sensors.centreOfMass.rows(0,2));
        arma::vec3 comGoal;
        //Select the COM position depending on stage
        if(stage == MotionStage::RUNNING){
            comGoal = arma::vec3({0, negativeIfRight * DarwinModel::Leg::FOOT_CENTRE_TO_ANKLE_CENTRE, stand_height});
        } else if(stage == MotionStage::STOPPING){
            comGoal = arma::vec3({0, - negativeIfRight * foot_separation / 2, stand_height});
        }
        comDiff = comGoal - centreOfMass_foot;

        Transform3D torsoTarget = torsoOrientation;
        torsoTarget.translation() += comDiff;

        float error = arma::norm(torsoPose.submat(0,3,2,3) - torsoTarget.submat(0,3,2,3));
        // std::cout << "error" << error << std::endl;
        // std::cout << "COM rel foot = " << centreOfMass_foot << std::endl;
        // std::cout << "comDiff  = " << comDiff << std::endl;
        // std::cout << "comGoal  = " << comGoal << std::endl;
        // std::cout << "torsoPose  = " << torsoPose << std::endl;
        // std::cout << "torsoTarget  = " << torsoTarget << std::endl;

        //WARNING: DO NOT SWAP stable CHECK AND newTorsoPose OR YOU WILL BREAK ROBOTS
        stable = error < tolerance;
        if(stable && stage == MotionStage::STOPPING) stage = MotionStage::FINISHED;
        //Interpolate both rotation and translation
        // Transform3D newTorsoPose = utility::math::matrix::Transform3D::interpolate(torsoPose, torsoTarget, deltaT * motion_gain);
        
        //Interpolate just translation
        float alpha = std::fmax(0,std::fmin(1,deltaT * motion_gain));
        Transform3D newTorsoPose = torsoTarget;
        newTorsoPose.translation() = alpha * (torsoTarget.translation() - torsoPose.translation()) + torsoPose.translation();

        std::cout << "newTorsoPose  = \n" << newTorsoPose << std::endl;
        //TODO: guard against invalid IK request
        return newTorsoPose.i();
    }

    Transform3D FootLifter::getFootPose(const Sensors& sensors, float){
        if(stage == MotionStage::RUNNING){
            
            double elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(sensors.timestamp - motionStartTime).count() * 1e-6;
            float alpha = std::fmax(0,std::fmin(velocity * elapsedTime / distance,1));
            stable = (alpha >= 1);
            return utility::math::matrix::Transform3D::interpolate(startPose,finishPose,alpha);

        } else if (stage == MotionStage::STOPPING) {
            
            double elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(sensors.timestamp - stoppingCommandTime).count() * 1e-6;
            float alpha = std::fmax(0,std::fmin(velocity * elapsedTime / distance,1));
            if(alpha >= 1) stage = MotionStage::FINISHED;
            return utility::math::matrix::Transform3D::interpolate(finishPose,startPose,alpha);
        }
        //Default
        return Transform3D();
	}

	Transform3D Kicker::getFootPose(const Sensors& sensors, float){
        if(stage == MotionStage::RUNNING){
            
            double elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(sensors.timestamp - motionStartTime).count() * 1e-6;
            float alpha = std::fmax(0,std::fmin(kick_velocity * elapsedTime / distance,1));
            if(alpha >= 1) stop();
            return utility::math::matrix::Transform3D::interpolate(startPose,finishPose,alpha);

        } else if (stage == MotionStage::STOPPING) {
            
            double elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(sensors.timestamp - stoppingCommandTime).count() * 1e-6;
            float alpha = std::fmax(0,std::fmin(return_velocity * elapsedTime / distance,1));
            if(alpha >= 1) stage = MotionStage::FINISHED;
            return utility::math::matrix::Transform3D::interpolate(finishPose,startPose,alpha);
        }
        //Default
        return Transform3D();
	}
}
}