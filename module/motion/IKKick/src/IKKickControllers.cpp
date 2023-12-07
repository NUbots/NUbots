/*
 * MIT License
 *
 * Copyright (c) 2015 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "IKKickControllers.hpp"

#include "message/actuation/KinematicsModel.hpp"


namespace module::motion {

    using extension::Configuration;

    using message::input::Sensors;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;
    using FrameID = utility::input::FrameID;
    using message::actuation::KinematicsModel;

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
        Eigen::Isometry3d torsoToFoot = getTorsoPose(sensors);
        Eigen::Isometry3d startPose   = torsoToFoot.inverse();

        int negativeIfRight          = (supportFoot == LimbID::RIGHT_LEG) ? -1 : 1;
        Eigen::Isometry3d finishPose = torsoToFoot;
        finishPose.translation() =
            Eigen::Vector3d(forward_lean,
                            negativeIfRight * (adjustment + kinematicsModel.leg.FOOT_CENTRE_TO_ANKLE_CENTRE),
                            stand_height);
        finishPose = finishPose.inverse();

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
        Eigen::Isometry3d startPose = Eigen::Isometry3d::Identity();

        // Convert torso to support foot
        Eigen::Isometry3d currentTorso = getTorsoPose(sensors);
        // Convert kick foot to torso
        Eigen::Isometry3d currentKickFoot = (supportFoot == LimbID::LEFT_LEG)
                                                ? Eigen::Isometry3d(sensors.Htx[FrameID::L_ANKLE_ROLL])
                                                : Eigen::Isometry3d(sensors.Htx[FrameID::R_ANKLE_ROLL]);

        // Convert support foot to kick foot coordinates = convert torso to kick foot * convert support foot to
        // torso
        Eigen::Isometry3d supportToKickFoot = currentKickFoot.inverse() * currentTorso.inverse();
        // Convert ball position from support foot coordinates to kick foot coordinates
        Eigen::Vector3d ballFromKickFoot = supportToKickFoot * ballPosition;
        Eigen::Vector3d goalFromKickFoot = supportToKickFoot * goalPosition;

        // Compute follow through:
        Eigen::Vector3d ballToGoalUnit = (goalFromKickFoot - ballFromKickFoot).normalized();
        Eigen::Vector3d followThrough  = follow_through * ballToGoalUnit;
        Eigen::Vector3d windUp         = -wind_up * ballToGoalUnit;

        // Get kick and lift goals
        Eigen::Vector3d kickGoal = followThrough;
        Eigen::Vector3d liftGoal = windUp;

        kickGoal.z() = kick_height;
        liftGoal.z() = kick_height;

        // constrain to prevent leg collision
        Eigen::Vector3d supportFootPos = supportToKickFoot.translation();
        int signSupportFootPosY        = supportFootPos.y() < 0 ? -1 : 1;
        float clippingPlaneY =
            supportFootPos.y()
            - signSupportFootPosY
                  * (foot_separation_margin
                     + (kinematicsModel.leg.FOOT_WIDTH / 2.0 - kinematicsModel.leg.FOOT_CENTRE_TO_ANKLE_CENTRE));

        float liftClipDistance = (liftGoal.y() - clippingPlaneY);
        if (signSupportFootPosY * liftClipDistance > 0) {
            // Clip
            liftGoal.topRows<2>() = liftGoal.topRows<2>() * clippingPlaneY / liftGoal.y();
        }

        float kickClipDistance = (kickGoal.y() - clippingPlaneY);
        if (signSupportFootPosY * kickClipDistance > 0) {
            // Clip
            kickGoal.topRows<2>() = kickGoal.topRows<2>() * clippingPlaneY / kickGoal.y();
        }

        kick.pose.translation()      = kickGoal;
        lift_foot.pose.translation() = liftGoal;

        kick.duration = (kickGoal - liftGoal).norm() / kick_velocity;

        // Robocup code / hacks
        auto startFrame                     = SixDOFFrame{startPose, 0};
        auto liftBeforeWindUp               = startFrame;
        liftBeforeWindUp.pose.translation() = Eigen::Vector3d(0, 0, lift_foot.pose.translation().z());
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
}  // namespace module::motion
