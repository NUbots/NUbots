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


namespace module::skill {

    using extension::Configuration;

    using message::input::Sensors;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;
    using FrameID = utility::input::FrameID;
    using message::actuation::KinematicsModel;

    void KickBalancer::configure(const Configuration& config) {
        cfg_controller.servo_angle_threshold = config["balancer"]["servo_angle_threshold"].as<float>();
        cfg_controller.forward_duration      = config["balancer"]["forward_duration"].as<float>();
        cfg_controller.return_duration       = config["balancer"]["return_duration"].as<float>();
        cfg_balancer.stand_height            = config["balancer"]["stand_height"].as<float>();
        cfg_balancer.forward_lean            = config["balancer"]["forward_lean"].as<float>();
        cfg_balancer.foot_separation         = config["balancer"]["foot_separation"].as<float>();
        cfg_balancer.adjustment              = config["balancer"]["adjustment"].as<float>();
    }

    void KickBalancer::compute_start_motion(const KinematicsModel& kinematics_model, const Sensors& sensors) {
        Eigen::Isometry3d torso_to_foot = get_torso_pose(sensors);
        Eigen::Isometry3d start_pose    = torso_to_foot.inverse();

        int negative_if_right         = (support_foot == LimbID::RIGHT_LEG) ? -1 : 1;
        Eigen::Isometry3d finish_pose = torso_to_foot;
        finish_pose.translation()     = Eigen::Vector3d(
            cfg_balancer.forward_lean,
            negative_if_right * (cfg_balancer.adjustment + kinematics_model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE),
            cfg_balancer.stand_height);
        finish_pose = finish_pose.inverse();

        std::vector<SixDOFFrame> frames;
        frames.push_back(SixDOFFrame{start_pose, 0});
        frames.push_back(SixDOFFrame{finish_pose, cfg_controller.forward_duration});
        anim = Animator(frames);
    }

    void KickBalancer::compute_stop_motion(const Sensors&) {
        // Play the reverse
        anim.frames[0].duration = cfg_controller.return_duration;
        std::reverse(anim.frames.begin(), anim.frames.end());
        anim.frames[0].duration = 0;
    }

    void Kicker::configure(const Configuration& config) {
        cfg_controller.servo_angle_threshold = config["kick_frames"]["servo_angle_threshold"].as<float>();
        lift_foot                            = SixDOFFrame(config["kick_frames"]["lift_foot"].config);
        kick                                 = SixDOFFrame(config["kick_frames"]["kick"].config);
        place_foot                           = SixDOFFrame(config["kick_frames"]["place_foot"].config);

        cfg_kicker.kick_velocity          = config["kick"]["kick_velocity"].as<float>();
        cfg_kicker.follow_through         = config["kick"]["follow_through"].as<float>();
        cfg_kicker.kick_height            = config["kick"]["kick_height"].as<float>();
        cfg_kicker.wind_up                = config["kick"]["wind_up"].as<float>();
        cfg_kicker.foot_separation_margin = config["kick"]["foot_separation_margin"].as<float>();

        cfg_kicker.lift_before_windup_duration  = config["kick"]["lift_before_windup_duration"].as<float>();
        cfg_kicker.return_before_place_duration = config["kick"]["return_before_place_duration"].as<float>();
    }

    void Kicker::compute_start_motion(const KinematicsModel& kinematics_model, const Sensors& sensors) {
        Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();

        // Convert torso to support foot
        Eigen::Isometry3d current_torso = get_torso_pose(sensors);
        // Convert kick foot to torso
        Eigen::Isometry3d current_kick_foot = (support_foot == LimbID::LEFT_LEG)
                                                  ? Eigen::Isometry3d(sensors.Htx[FrameID::L_ANKLE_ROLL])
                                                  : Eigen::Isometry3d(sensors.Htx[FrameID::R_ANKLE_ROLL]);

        // Convert support foot to kick foot coordinates = convert torso to kick foot * convert support foot to
        // torso
        Eigen::Isometry3d Hks = current_kick_foot.inverse() * current_torso.inverse();
        // Convert ball position from support foot coordinates to kick foot coordinates
        Eigen::Vector3d rBKk = Hks * ball_position;
        Eigen::Vector3d rGKk = Hks * goal_position;

        // Compute follow through:
        Eigen::Vector3d uGBk           = (rGKk - rBKk).normalized();
        Eigen::Vector3d follow_through = cfg_kicker.follow_through * uGBk;
        Eigen::Vector3d windup         = -cfg_kicker.wind_up * uGBk;

        // Get kick and lift goals
        Eigen::Vector3d kick_goal = follow_through;
        Eigen::Vector3d lift_goal = windup;

        kick_goal.z() = cfg_kicker.kick_height;
        lift_goal.z() = cfg_kicker.kick_height;

        // constrain to prevent leg collision
        Eigen::Vector3d support_foot_pos = Hks.translation();
        int sign_support_foot_pos_y      = support_foot_pos.y() < 0 ? -1 : 1;
        float clipping_plane_y =
            support_foot_pos.y()
            - sign_support_foot_pos_y
                  * (cfg_kicker.foot_separation_margin
                     + (kinematics_model.leg.FOOT_WIDTH / 2.0 - kinematics_model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE));

        float lift_clip_distance = (lift_goal.y() - clipping_plane_y);
        if (sign_support_foot_pos_y * lift_clip_distance > 0) {
            // Clip
            lift_goal.topRows<2>() = lift_goal.topRows<2>() * clipping_plane_y / lift_goal.y();
        }

        float kick_clip_distance = (kick_goal.y() - clipping_plane_y);
        if (sign_support_foot_pos_y * kick_clip_distance > 0) {
            // Clip
            kick_goal.topRows<2>() = kick_goal.topRows<2>() * clipping_plane_y / kick_goal.y();
        }

        kick.pose.translation()      = kick_goal;
        lift_foot.pose.translation() = lift_goal;

        kick.duration = (kick_goal - lift_goal).norm() / cfg_kicker.kick_velocity;

        // Robocup code / hacks
        auto start_frame                      = SixDOFFrame{start_pose, 0};
        auto lift_before_windup               = start_frame;
        lift_before_windup.pose.translation() = Eigen::Vector3d(0, 0, lift_foot.pose.translation().z());
        lift_before_windup.duration           = cfg_kicker.lift_before_windup_duration;
        auto return_before_place              = lift_before_windup;
        return_before_place.duration          = cfg_kicker.return_before_place_duration;

        std::vector<SixDOFFrame> frames;
        frames.push_back(start_frame);
        frames.push_back(lift_before_windup);
        frames.push_back(lift_foot);
        frames.push_back(kick);
        frames.push_back(return_before_place);
        frames.push_back(place_foot);
        anim = Animator(frames);
    }
    void Kicker::compute_stop_motion(const Sensors&) {
        // Just play the place_foot frame
        std::vector<SixDOFFrame> frames;
        frames.push_back(place_foot);
        anim = Animator(frames);
    }
}  // namespace module::skill
