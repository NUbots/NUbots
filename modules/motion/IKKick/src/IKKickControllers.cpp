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
        adjustment = config["balancer"]["adjustment"].as<float>();
        forward_duration = config["balancer"]["forward_duration"].as<float>();
        return_duration = config["balancer"]["return_duration"].as<float>();
	}

	void FootLifter::configure(const Configuration<IKKickConfig>& config){
        lift_foot_height = config["lifter"]["lift_foot_height"].as<float>();
        put_foot_down_height = config["lifter"]["put_foot_down_height"].as<float>();
        lift_foot_back = config["lifter"]["lift_foot_back"].as<float>();
        
        forward_duration = config["lifter"]["forward_duration"].as<float>();
        return_duration = config["lifter"]["return_duration"].as<float>();
	}

	void Kicker::configure(const Configuration<IKKickConfig>& config) {
        forward_duration = config["kicker"]["forward_duration"].as<float>();
        return_duration = config["kicker"]["return_duration"].as<float>();
	}

    void KickBalancer::computeStartMotion(const Sensors& sensors) {
        Transform3D torsoToFoot = getTorsoPose(sensors);
        Transform3D startPose = torsoToFoot.i();

        int negativeIfRight = (supportFoot == LimbID::RIGHT_LEG) ? -1 : 1;
        Transform3D finishPose = torsoToFoot;
        finishPose.translation() = arma::vec3({forward_lean, negativeIfRight * (adjustment + DarwinModel::Leg::FOOT_CENTRE_TO_ANKLE_CENTRE), stand_height});
        finishPose = finishPose.i();

        std::vector<SixDOFFrame> frames;
        frames.push_back(SixDOFFrame{startPose,0});
        frames.push_back(SixDOFFrame{finishPose,forward_duration});
        anim = Animator(frames);
    }

    void FootLifter::computeStartMotion(const Sensors&) {
        Transform3D startPose = arma::eye(4,4);
        Transform3D finishPose = startPose.translate(arma::vec3({-lift_foot_back,0,lift_foot_height}));
        
        std::vector<SixDOFFrame> frames;
        frames.push_back(SixDOFFrame{startPose,0});
        frames.push_back(SixDOFFrame{finishPose,forward_duration});
        anim = Animator(frames);
    }


    void Kicker::computeStartMotion(const Sensors& sensors) {
        Transform3D startPose = arma::eye(4,4);

        // Convert torso to support foot

        Transform3D currentTorso = getTorsoPose(sensors);
        // Convert kick foot to torso 
        Transform3D currentKickFoot = supportFoot == LimbID::LEFT_LEG ? sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second 
                                                                      : sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
        // Convert support foot to kick foot coordinates = convert torso to kick foot * convert support foot to torso
        Transform3D supportToKickFoot = currentKickFoot.i() * currentTorso.i();
        // Convert ball position from support foot coordinates to kick foot coordinates
        arma::vec3 ballFromKickFoot = supportToKickFoot.transformPoint(ballPosition);
        Transform3D finishPose = startPose.translate(ballFromKickFoot);
        
        std::vector<SixDOFFrame> frames;
        frames.push_back(SixDOFFrame{startPose,0});
        frames.push_back(SixDOFFrame{finishPose,forward_duration});
        anim = Animator(frames);

    }


    void KickBalancer::computeStopMotion(const Sensors&){
        //EWW HACKS
        anim.frames[0].duration = return_duration;
        std::reverse(anim.frames.begin(),anim.frames.end());
        anim.frames[0].duration = 0;
    }

    void FootLifter::computeStopMotion(const Sensors&){
        //EWW HACKS
        anim.frames[0].pose.translate(arma::vec3({0,0,put_foot_down_height}));
        anim.frames[0].duration = return_duration;
        std::reverse(anim.frames.begin(),anim.frames.end());
        anim.frames[0].duration = 0;
    }
    
    void Kicker::computeStopMotion(const Sensors&){
        //EWW HACKS
        anim.frames[0].duration = return_duration;
        std::reverse(anim.frames.begin(),anim.frames.end());
        anim.frames[0].duration = 0;
    }
}
}