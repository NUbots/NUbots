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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "IKKickControllers.h"

#include "message/motion/KinematicsModel.h"

namespace module {
namespace motion {

    using extension::Configuration;

    using message::input::Sensors;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;
    using message::motion::KinematicsModel;

    using utility::math::matrix::Transform3D;

    void KickBalancer::configure(const Configuration& config) {
        servo_angle_threshold = config["balancer"]["servo_angle_threshold"].as<float>();
        stand_height          = config["balancer"]["stand_height"].as<float>();
        forward_lean          = config["balancer"]["forward_lean"].as<float>();
        foot_separation       = config["balancer"]["foot_separation"].as<float>();
        adjustment            = config["balancer"]["adjustment"].as<float>();
        forward_duration      = config["balancer"]["forward_duration"].as<float>();
        return_duration       = config["balancer"]["return_duration"].as<float>();
    }

    void KickBalancer::computeStartMotion(const KinematicsModel& kinematicsModel, const Sensors& sensors) {
        Transform3D torsoToFoot = getTorsoPose(sensors);
        Transform3D startPose   = torsoToFoot.i();

        int negativeIfRight    = (supportFoot == LimbID::RIGHT_LEG) ? -1 : 1;
        Transform3D finishPose = torsoToFoot;
        finishPose.translation() =
            arma::vec3({forward_lean,
                        negativeIfRight * (adjustment + kinematicsModel.leg.FOOT_CENTRE_TO_ANKLE_CENTRE),
                        stand_height});
        finishPose = finishPose.i();

        std::vector<SixDOFFrame> frames;
        frames.push_back(SixDOFFrame{startPose, 0});
        frames.push_back(SixDOFFrame{finishPose, forward_duration});
        anim = Animator(frames);
    }

    void KickBalancer::computeStopMotion(const Sensors&) {
        // Play the reverse
        anim.frames[0].duration = return_duration;
        std::reverse(anim.frames.begin(), anim.frames.end());
        anim.frames[0].duration = 0;
    }

    void Kicker::configure(const Configuration& config) {
        servo_angle_threshold = config["kick_frames"]["servo_angle_threshold"].as<float>();
        lift_foot             = SixDOFFrame(config["kick_frames"]["lift_foot"].config);
        kick                  = SixDOFFrame(config["kick_frames"]["kick"].config);
        place_foot            = SixDOFFrame(config["kick_frames"]["place_foot"].config);

        kick_velocity          = config["kick"]["kick_velocity"].as<float>();
        follow_through         = config["kick"]["follow_through"].as<float>();
        kick_height            = config["kick"]["kick_height"].as<float>();
        wind_up                = config["kick"]["wind_up"].as<float>();
        foot_separation_margin = config["kick"]["foot_separation_margin"].as<float>();

        lift_before_windup_duration  = config["kick"]["lift_before_windup_duration"].as<float>();
        return_before_place_duration = config["kick"]["return_before_place_duration"].as<float>();
    }

    void Kicker::computeStartMotion(const KinematicsModel& kinematicsModel, const Sensors& sensors) {
        Transform3D startPose = arma::eye(4, 4);

        // Convert torso to support foot
        Transform3D currentTorso = getTorsoPose(sensors);
        // Convert kick foot to torso
        Transform3D currentKickFoot = (supportFoot == LimbID::LEFT_LEG)
                                          ? convert(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL])
                                          : convert(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);

        // Convert support foot to kick foot coordinates = convert torso to kick foot * convert support foot to torso
        Transform3D supportToKickFoot = currentKickFoot.i() * currentTorso.i();
        // Convert ball position from support foot coordinates to kick foot coordinates
        arma::vec3 ballFromKickFoot = supportToKickFoot.transformPoint(ballPosition);
        arma::vec3 goalFromKickFoot = supportToKickFoot.transformPoint(goalPosition);

        // Compute follow through:
        arma::vec3 ballToGoalUnit = arma::normalise(goalFromKickFoot - ballFromKickFoot);
        arma::vec3 followThrough  = follow_through * ballToGoalUnit;
        arma::vec3 windUp         = -wind_up * ballToGoalUnit;

        // Get kick and lift goals
        arma::vec3 kickGoal = followThrough;
        arma::vec3 liftGoal = windUp;

        kickGoal[2] = kick_height;
        liftGoal[2] = kick_height;

        // constrain to prevent leg collision
        arma::vec3 supportFootPos = supportToKickFoot.translation();
        int signSupportFootPosY   = supportFootPos[1] < 0 ? -1 : 1;
        float clippingPlaneY =
            supportFootPos[1]
            - signSupportFootPosY
                  * (foot_separation_margin
                     + (kinematicsModel.leg.FOOT_WIDTH / 2.0 - kinematicsModel.leg.FOOT_CENTRE_TO_ANKLE_CENTRE));

        float liftClipDistance = (liftGoal[1] - clippingPlaneY);
        if (signSupportFootPosY * liftClipDistance > 0) {
            // Clip
            liftGoal.rows(0, 1) = liftGoal.rows(0, 1) * clippingPlaneY / liftGoal[1];
        }

        float kickClipDistance = (kickGoal[1] - clippingPlaneY);
        if (signSupportFootPosY * kickClipDistance > 0) {
            // Clip
            kickGoal.rows(0, 1) = kickGoal.rows(0, 1) * clippingPlaneY / kickGoal[1];
        }

        kick.pose.translation()      = kickGoal;
        lift_foot.pose.translation() = liftGoal;

        kick.duration = arma::norm(kickGoal - liftGoal) / kick_velocity;

        // Robocup code / hacks
        auto startFrame                     = SixDOFFrame{startPose, 0};
        auto liftBeforeWindUp               = startFrame;
        liftBeforeWindUp.pose.translation() = arma::vec3{0, 0, lift_foot.pose.translation()[2]};
        liftBeforeWindUp.duration           = lift_before_windup_duration;
        auto returnBeforePlace              = liftBeforeWindUp;
        returnBeforePlace.duration          = return_before_place_duration;

        std::vector<SixDOFFrame> frames;
        frames.push_back(startFrame);
        frames.push_back(liftBeforeWindUp);
        frames.push_back(lift_foot);
        frames.push_back(kick);
        frames.push_back(returnBeforePlace);
        frames.push_back(place_foot);
        anim = Animator(frames);
    }
    void Kicker::computeStopMotion(const Sensors&) {
        // Just play the place_foot frame
        std::vector<SixDOFFrame> frames;
        frames.push_back(place_foot);
        anim = Animator(frames);
    }
}  // namespace motion
}  // namespace module
