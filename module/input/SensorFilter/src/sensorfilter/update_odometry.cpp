/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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
#include "SensorFilter.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/math/filter/KalmanFilter.hpp"
#include "utility/math/filter/MahonyFilter.hpp"
#include "utility/math/filter/YawFilter.hpp"

#include <nlopt.hpp>
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::platform::RawSensors;
    using message::input::StellaMap;

    using utility::input::FrameID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;

    using utility::math::filter::KalmanFilter;
    using utility::math::filter::MahonyFilter;
    using utility::math::filter::YawFilter;

    void SensorFilter::update_odometry(std::unique_ptr<Sensors>& sensors,
                                       const std::shared_ptr<const Sensors>& previous_sensors,
                                       const RawSensors& raw_sensors,
                                       const message::behaviour::state::Stability& stability,
                                       const std::shared_ptr<const RobotPoseGroundTruth>& robot_pose_ground_truth,
                                       const std::shared_ptr<const StellaMsg>& stella) {

        // Simple fault detection (you can make this more sophisticated later)
        FaultDetectionStatus fault_status;

        // Check Stella tracking state
        bool stella_actively_tracking = false;
        if (stella && stella_state == StellaState::INITIALIZED) {
            stella_actively_tracking = (stella->tracking_state == "Tracking");
            log<INFO>("Stella tracking state: '" + stella->tracking_state + "', healthy: " + std::to_string(stella_actively_tracking));
        }

        fault_status.stella_healthy = stella_actively_tracking;
        fault_status.kinematics_healthy = stability > Stability::FALLING;
        fault_status.stella_confidence = fault_status.stella_healthy ? 1.0 : 0.0;
        fault_status.kinematics_confidence = fault_status.kinematics_healthy ? 1.0 : 0.0;

        // Check if sliding window is enabled
        if (cfg.sliding_window.enabled) {
            update_odometry_sliding_window(sensors, previous_sensors, raw_sensors,
                                         stability, robot_pose_ground_truth,
                                         stella, fault_status);
            return;
        }


        // Use ground truth instead of calculating odometry, then return
        if (cfg.use_ground_truth && robot_pose_ground_truth) {
            Eigen::Isometry3d Hft = Eigen::Isometry3d(robot_pose_ground_truth->Hft);
            if (!ground_truth_initialised) {
                // Initialise the ground truth Hfw
                ground_truth_Hfw.translation().head<2>() = Hft.translation().head<2>();
                ground_truth_Hfw.translation()[2]        = 0;
                double yaw                               = mat_to_rpy_intrinsic(Hft.rotation()).z();
                ground_truth_Hfw.linear()                = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, yaw));
                ground_truth_initialised                 = true;
            }

            // Construct world {w} to torso {t} space transform from ground truth pose
            sensors->Htw = Eigen::Isometry3d(Hft).inverse() * ground_truth_Hfw;
            // Construct robot {r} to world {w} space transform from ground truth
            Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
            auto Hwt              = sensors->Htw.inverse();
            Hwr.linear() =
                Eigen::AngleAxisd(mat_to_rpy_intrinsic(Hwt.linear()).z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Hwr.translation() = Eigen::Vector3d(Hwt.translation().x(), Hwt.translation().y(), 0.0);
            sensors->Hrw      = Hwr.inverse();
            sensors->vTw      = Eigen::Vector3d(robot_pose_ground_truth->vTf);
            return;
        }
        else if (cfg.use_ground_truth && !robot_pose_ground_truth) {
            log<WARN>("No ground truth data received, but use_ground_truth is true");
        }

        // Compute time since last update
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // Adapt Mahony filter Kp gain based on stability state
        if (stability == Stability::STANDING) {
            mahony_filter.set_Kp(cfg.adaptive_gains.standing_Kp);
        }
        else {
            mahony_filter.set_Kp(cfg.adaptive_gains.dynamic_Kp);
        }

        // Perform Mahony update
        const auto Rwt_mahony      = mahony_filter.update(sensors->accelerometer, sensors->gyroscope, dt);
        Eigen::Vector3d rpy_mahony = mat_to_rpy_intrinsic(Rwt_mahony);

        // If fallen, keep position still
        if (stability <= Stability::FALLING) {
            Eigen::Isometry3d Hwt =
                previous_sensors == nullptr ? Eigen::Isometry3d::Identity() : previous_sensors->Htw.inverse();

            // Update yaw filter with gyroscope only (no kinematic measurement available when fallen)
            const double fallen_yaw = yaw_filter.update_gyro_only(sensors->gyroscope.z(), dt);

            // Htw rotation is combination of Mahony pitch and roll and filtered yaw
            Eigen::Vector3d rpy_fallen(rpy_mahony.x(), rpy_mahony.y(), fallen_yaw);
            Hwt.linear() = rpy_intrinsic_to_mat(rpy_fallen);
            sensors->Htw = Hwt.inverse();

            // Get robot to world
            sensors->Hrw = previous_sensors == nullptr ? Eigen::Isometry3d::Identity() : previous_sensors->Hrw;

            // Set velocity to zero
            sensors->vTw = Eigen::Vector3d::Zero();

            return;
        }

        // If sensors detected a new foot phase, update the anchor frame
        if (planted_anchor_foot != sensors->planted_foot_phase
            && sensors->planted_foot_phase != WalkState::Phase::DOUBLE) {
            switch (planted_anchor_foot.value) {
                case WalkState::Phase::RIGHT:
                    Hwp = Hwp * sensors->Htx[FrameID::R_FOOT_BASE].inverse() * sensors->Htx[FrameID::L_FOOT_BASE];
                    break;
                case WalkState::Phase::LEFT:
                    Hwp = Hwp * sensors->Htx[FrameID::L_FOOT_BASE].inverse() * sensors->Htx[FrameID::R_FOOT_BASE];
                    break;
                default: log<WARN>("Anchor frame should not be updated in double support phase"); break;
            }
            planted_anchor_foot = sensors->planted_foot_phase;
            // Set the z translation, roll and pitch of the anchor frame to 0 as known to be on field plane
            Hwp.translation().z() = 0;
            Hwp.linear()          = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, mat_to_rpy_intrinsic(Hwp.linear()).z()));
        }
        sensors->Hwp = Hwp;

        // Compute torso pose using kinematics from anchor frame (current planted foot)
        const Eigen::Isometry3d Hpt        = planted_anchor_foot.value == WalkState::Phase::RIGHT
                                                 ? Eigen::Isometry3d(sensors->Htx[FrameID::R_FOOT_BASE].inverse())
                                                 : Eigen::Isometry3d(sensors->Htx[FrameID::L_FOOT_BASE].inverse());
        const Eigen::Isometry3d Hwt_anchor = Hwp * Hpt;

        // Extract yaw from kinematic estimate
        const double kinematic_yaw = mat_to_rpy_intrinsic(Hwt_anchor.linear()).z();

        // Fuse yaw estimates using yaw filter
        const double fused_yaw = yaw_filter.update(sensors->gyroscope.z(), kinematic_yaw, dt);

        // Construct world {w} to torso {t} space transform (mahony roll/pitch, fused yaw, anchor translation)
        Eigen::Isometry3d Hwt = Eigen::Isometry3d::Identity();
        Hwt.translation()     = Hwt_anchor.translation();

        // Combine Mahony roll/pitch with fused yaw
        Eigen::Vector3d rpy_fused(rpy_mahony.x(), rpy_mahony.y(), fused_yaw);
        Hwt.linear() = rpy_intrinsic_to_mat(rpy_fused);
        Eigen::Isometry3d Htw_mahony = Hwt.inverse();


    // Handle Stella initialization state transitions
    if (stella && stella->map_points.size() > 0) {
        if (stella_state == StellaState::WAITING_FOR_POINTS) {
            stella_state = StellaState::WAITING_FOR_DELAY;
            stella_start_time = std::chrono::steady_clock::now();
            log<INFO>("Received first stella points");
        } else if (stella_state == StellaState::WAITING_FOR_DELAY) {
            double time_since_first_points = std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::steady_clock::now() - *stella_start_time).count();
            if (time_since_first_points > cfg.stella_config.initialization_delay)
            {
                Eigen::Isometry3d Htf = Eigen::Isometry3d(sensors->Htx[FrameID::L_FOOT_BASE]);
                // Remove y translation from Htf
                auto Htw = Htf;
                Htw.translation().y() = 0;

                Eigen::Isometry3d Htc = Eigen::Isometry3d(sensors->Htx[FrameID::L_CAMERA]);
                auto Hwc = Htw.inverse() * Htc;
                // 90 degrees y, -90 degrees z
                auto Hcn = Eigen::Isometry3d::Identity();
                Hcn.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                Hwn = Hwc * Hcn;

                stella_state = StellaState::INITIALIZED;
                log<INFO>("Stella initialised");
            }
        }
    }

    if (stella && stella_state == StellaState::INITIALIZED) {
        auto Htc =  Eigen::Isometry3d(sensors->Htx[FrameID::L_CAMERA]);
        auto Hwc = Hwt_anchor * Htc;
        // Get the map points from stella
        std::vector<Eigen::Vector3d> rPNn_points = stella->map_points;

        // Transform the map points into the world frame and project onto the field plane
        std::vector<Eigen::Vector3d> rPWw_stella;
        std::vector<Eigen::Vector3d> rPCw_stella;
        std::vector<Eigen::Vector3d> rPWw_ground;
        std::vector<Eigen::Vector3d> rPCw_ground;
        for (const auto& rPNn : rPNn_points) {
            auto rPWw = Hwn * rPNn;
            rPWw_stella.push_back(rPWw);
            auto rPCw = rPWw - Hwc.translation();
            rPCw_stella.push_back(rPCw);

            // Project onto the field plane
            auto uPCw = rPCw / rPCw.norm();
            auto rPCw_g = uPCw * std::abs(Hwc.translation().z() / uPCw.z());
            auto rPWw_g = rPCw_g + Hwc.translation();
            rPWw_ground.push_back(rPWw_g);
            rPCw_ground.push_back(rPCw_g);
        }

        // Calculate the scale factor between the map points and the ground points,
        // but only consider points whose ground-projected distance is less than 2m
        double new_scale_factor = 1.0;
        size_t count = 0;
        std::vector<Eigen::Vector3d> rPWw_accepted;
        for (size_t i = 0; i < rPCw_stella.size(); i++) {
            // Only consider points that are less than 2m away, have a non-zero distance, and are on the ground
            if (rPCw_ground[i].norm() < 2.0 && rPCw_stella[i].norm() > 1e-6 && rPWw_ground[i].z() < 1e-4) { // avoid divide by zero
                new_scale_factor += rPCw_ground[i].norm() / rPCw_stella[i].norm();
                count++;
                rPWw_accepted.push_back(rPWw_ground[i]);
            }
        }
        if (count > 0) {
            new_scale_factor /= count;
        }

        // Exponential filter the scale factor
        scale_factor = (1 - cfg.stella_config.scale_factor_alpha) * scale_factor + cfg.stella_config.scale_factor_alpha * new_scale_factor;

        auto stella_map = std::make_unique<StellaMap>();
        std::vector<Eigen::Vector3d> rPWw_map;
        for (size_t i = 0; i < rPCw_stella.size(); i++) {
            Eigen::Vector3d rPWw_scaled = Hwc.translation() + rPCw_stella[i] * scale_factor;
            rPWw_map.push_back(rPWw_scaled);
        }
        stella_map->rPWw_map = rPWw_map;
        emit(std::move(stella_map));


        // Adjust the Hnc transform to account for the scale factor
        auto Hnc = stella->Hnc;
        Hnc.translation() = Hnc.translation() * scale_factor;

        // Construct the Htw transform using the stella pose
        auto Htw_stella = Htc * Hnc.inverse() * Hwn.inverse();

        // Apply constraints: Force Stella to use kinematic z + IMU roll/pitch
        auto Hwt_stella = Htw_stella.inverse();  // Get world-to-torso transform from Stella
        Eigen::Vector3d rpy_imu = mat_to_rpy_intrinsic(Rwt_mahony);  // Get IMU roll/pitch from Mahony filter
        Eigen::Vector3d rpy_stella = mat_to_rpy_intrinsic(Hwt_stella.linear());  // Get Stella's original attitude

        // Create constrained transform: Stella's x,y,yaw + kinematic z + IMU roll/pitch
        Eigen::Isometry3d Hwt_constrained = Eigen::Isometry3d::Identity();

        // Keep Stella's x, y position
        Hwt_constrained.translation().x() = Hwt_stella.translation().x();
        Hwt_constrained.translation().y() = Hwt_stella.translation().y();

        // Apply constraints:
        // - z from kinematics: g_z(z) = z_stella - z_kinematic = 0
        Hwt_constrained.translation().z() = Hwt_anchor.translation().z();

        // - roll/pitch from IMU: g_roll(roll) = roll_stella - roll_imu = 0
        //                        g_pitch(pitch) = pitch_stella - pitch_imu = 0
        // - yaw from Stella (preserve Stella's yaw estimate)
        Eigen::Vector3d rpy_constrained(rpy_imu.x(),      // Roll from IMU
                                       rpy_imu.y(),      // Pitch from IMU
                                       rpy_stella.z());  // Yaw from Stella
        Hwt_constrained.linear() = rpy_intrinsic_to_mat(rpy_constrained);

        sensors->Htw = Hwt_constrained.inverse();

        // Emit debug graphs for monitoring
        emit(graph("Roll IMU", rpy_imu.x()));
        emit(graph("Pitch IMU", rpy_imu.y()));
        emit(graph("Roll stella original", rpy_stella.x()));
        emit(graph("Pitch stella original", rpy_stella.y()));
        emit(graph("Roll constrained", rpy_constrained.x()));
        emit(graph("Pitch constrained", rpy_constrained.y()));
        emit(graph("Z anchor (kinematic)", Hwt_anchor.translation().z()));
        emit(graph("Z stella original", Hwt_stella.translation().z()));
        emit(graph("Z constrained", Hwt_constrained.translation().z()));
        emit(graph("Roll constraint error", rpy_stella.x() - rpy_imu.x()));
        emit(graph("Pitch constraint error", rpy_stella.y() - rpy_imu.y()));
        emit(graph("Z constraint error", Hwt_stella.translation().z() - Hwt_anchor.translation().z()));
        sensors->Hwn = Hwn;

        // Construct robot {r} to world {w} space transform (just x-y translation and fused yaw rotation)
        auto Hwt = sensors->Htw;
        Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
        Hwr.linear()          = Eigen::AngleAxisd(mat_to_rpy_intrinsic(Hwt.linear()).z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Hwr.translation()     = Eigen::Vector3d(Hwt.translation().x(), Hwt.translation().y(), 0.0);
        sensors->Hrw          = Hwr.inverse();

        // Low pass filter for torso velocity
        const double y_current     = Hwt.translation().y();
        const double y_prev        = previous_sensors ? previous_sensors->Htw.inverse().translation().y() : y_current;
        const double y_dot_current = (y_current - y_prev) / dt;
        const double y_dot =
            (dt / cfg.y_cut_off_frequency) * y_dot_current + (1 - (dt / cfg.y_cut_off_frequency)) * sensors->vTw.y();
        const double x_current     = Hwt.translation().x();
        const double x_prev        = previous_sensors ? previous_sensors->Htw.inverse().translation().x() : x_current;
        const double x_dot_current = (x_current - x_prev) / dt;
        const double x_dot =
            (dt / cfg.x_cut_off_frequency) * x_dot_current + (1 - (dt / cfg.x_cut_off_frequency)) * sensors->vTw.x();
        sensors->vTw = Eigen::Vector3d(x_dot, y_dot, 0);
    }
}

void SensorFilter::update_odometry_sliding_window(
    std::unique_ptr<Sensors>& sensors,
    const std::shared_ptr<const Sensors>& previous_sensors,
    const RawSensors& raw_sensors,
    const message::behaviour::state::Stability& stability,
    const std::shared_ptr<const RobotPoseGroundTruth>& robot_pose_ground_truth,
    const std::shared_ptr<const StellaMsg>& stella,
    const FaultDetectionStatus& fault_status) {

    log<INFO>("Using sliding window odometry with optimization");

    // Handle ground truth case
    if (cfg.use_ground_truth && robot_pose_ground_truth) {
        Eigen::Isometry3d Hft = Eigen::Isometry3d(robot_pose_ground_truth->Hft);
        if (!ground_truth_initialised) {
            ground_truth_Hfw.translation().head<2>() = Hft.translation().head<2>();
            ground_truth_Hfw.translation()[2]        = 0;
            double yaw                               = mat_to_rpy_intrinsic(Hft.rotation()).z();
            ground_truth_Hfw.linear()                = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, yaw));
            ground_truth_initialised                 = true;
        }
        sensors->Htw = Eigen::Isometry3d(Hft).inverse() * ground_truth_Hfw;
        Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
        auto Hwt              = sensors->Htw.inverse();
        Hwr.linear() = Eigen::AngleAxisd(mat_to_rpy_intrinsic(Hwt.linear()).z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Hwr.translation() = Eigen::Vector3d(Hwt.translation().x(), Hwt.translation().y(), 0.0);
        sensors->Hrw      = Hwr.inverse();
        sensors->vTw      = Eigen::Vector3d(robot_pose_ground_truth->vTf);
        return;
    }

    // Compute dt
    const double dt = std::max(
        std::chrono::duration_cast<std::chrono::duration<double>>(
            raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
            .count(),
        0.0);

    // Run IMU updates
    if (stability == Stability::STANDING) {
        mahony_filter.set_Kp(cfg.adaptive_gains.standing_Kp);
    } else {
        mahony_filter.set_Kp(cfg.adaptive_gains.dynamic_Kp);
    }
    const auto Rwt_mahony = mahony_filter.update(sensors->accelerometer, sensors->gyroscope, dt);
    Eigen::Vector3d rpy_mahony = mat_to_rpy_intrinsic(Rwt_mahony);

    // Handle fallen state
    if (stability <= Stability::FALLING) {
        Eigen::Isometry3d Hwt = previous_sensors == nullptr ? Eigen::Isometry3d::Identity() : previous_sensors->Htw.inverse();
        const double fallen_yaw = yaw_filter.update_gyro_only(sensors->gyroscope.z(), dt);
        Eigen::Vector3d rpy_fallen(rpy_mahony.x(), rpy_mahony.y(), fallen_yaw);
        Hwt.linear() = rpy_intrinsic_to_mat(rpy_fallen);
        sensors->Htw = Hwt.inverse();
        sensors->Hrw = previous_sensors == nullptr ? Eigen::Isometry3d::Identity() : previous_sensors->Hrw;
        sensors->vTw = Eigen::Vector3d::Zero();
        return;
    }

    // ===== SLIDING WINDOW OPTIMIZATION IMPLEMENTATION =====

    // 1. Compute kinematic estimate (always needed as potential measurement)
    if (planted_anchor_foot != sensors->planted_foot_phase && sensors->planted_foot_phase != WalkState::Phase::DOUBLE) {
        switch (planted_anchor_foot.value) {
            case WalkState::Phase::RIGHT:
                Hwp = Hwp * sensors->Htx[FrameID::R_FOOT_BASE].inverse() * sensors->Htx[FrameID::L_FOOT_BASE];
                break;
            case WalkState::Phase::LEFT:
                Hwp = Hwp * sensors->Htx[FrameID::L_FOOT_BASE].inverse() * sensors->Htx[FrameID::R_FOOT_BASE];
                break;
            default: break;
        }
        planted_anchor_foot = sensors->planted_foot_phase;
        Hwp.translation().z() = 0;
        Hwp.linear() = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, mat_to_rpy_intrinsic(Hwp.linear()).z()));
    }
    sensors->Hwp = Hwp;

    const Eigen::Isometry3d Hpt = planted_anchor_foot.value == WalkState::Phase::RIGHT
                                     ? Eigen::Isometry3d(sensors->Htx[FrameID::R_FOOT_BASE].inverse())
                                     : Eigen::Isometry3d(sensors->Htx[FrameID::L_FOOT_BASE].inverse());
    const Eigen::Isometry3d Hwt_kinematic = Hwp * Hpt;

    // 2. Compute Stella estimate (if available and healthy)
    Eigen::Isometry3d Hwt_stella = Eigen::Isometry3d::Identity();
    bool stella_available = false;

    // Handle Stella initialization (copied from your original logic)
    if (stella && stella->map_points.size() > 0) {
        if (stella_state == StellaState::WAITING_FOR_POINTS) {
            stella_state = StellaState::WAITING_FOR_DELAY;
            stella_start_time = std::chrono::steady_clock::now();
            log<INFO>("Received first stella points");
        } else if (stella_state == StellaState::WAITING_FOR_DELAY) {
            double time_since_first_points = std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::steady_clock::now() - *stella_start_time).count();
            if (time_since_first_points > cfg.stella_config.initialization_delay) {
                Eigen::Isometry3d Htf = Eigen::Isometry3d(sensors->Htx[FrameID::L_FOOT_BASE]);
                auto Htw = Htf;
                Htw.translation().y() = 0;
                Eigen::Isometry3d Htc = Eigen::Isometry3d(sensors->Htx[FrameID::L_CAMERA]);
                auto Hwc = Htw.inverse() * Htc;
                auto Hcn = Eigen::Isometry3d::Identity();
                Hcn.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                               Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                Hwn = Hwc * Hcn;
                stella_state = StellaState::INITIALIZED;
                log<INFO>("Stella initialised");
            }
        }
    }

    // Compute Stella pose if initialized and healthy
    if (stella && stella_state == StellaState::INITIALIZED && fault_status.stella_healthy) {
        auto Htc = Eigen::Isometry3d(sensors->Htx[FrameID::L_CAMERA]);
        auto Hwc = Hwt_kinematic * Htc;  // Use kinematic as base for camera position

        // Get the map points from stella (FULL IMPLEMENTATION FROM ORIGINAL)
        std::vector<Eigen::Vector3d> rPNn_points = stella->map_points;

        // Transform the map points into the world frame and project onto the field plane
        std::vector<Eigen::Vector3d> rPWw_stella;
        std::vector<Eigen::Vector3d> rPCw_stella;
        std::vector<Eigen::Vector3d> rPWw_ground;
        std::vector<Eigen::Vector3d> rPCw_ground;
        for (const auto& rPNn : rPNn_points) {
            auto rPWw = Hwn * rPNn;
            rPWw_stella.push_back(rPWw);
            auto rPCw = rPWw - Hwc.translation();
            rPCw_stella.push_back(rPCw);

            // Project onto the field plane
            auto uPCw = rPCw / rPCw.norm();
            auto rPCw_g = uPCw * std::abs(Hwc.translation().z() / uPCw.z());
            auto rPWw_g = rPCw_g + Hwc.translation();
            rPWw_ground.push_back(rPWw_g);
            rPCw_ground.push_back(rPCw_g);
        }

        // Calculate the scale factor between the map points and the ground points,
        // but only consider points whose ground-projected distance is less than 2m
        double new_scale_factor = 1.0;
        size_t count = 0;
        std::vector<Eigen::Vector3d> rPWw_accepted;
        for (size_t i = 0; i < rPCw_stella.size(); i++) {
            // Only consider points that are less than 2m away, have a non-zero distance, and are on the ground
            if (rPCw_ground[i].norm() < 2.0 && rPCw_stella[i].norm() > 1e-6 && rPWw_ground[i].z() < 1e-4) {
                new_scale_factor += rPCw_ground[i].norm() / rPCw_stella[i].norm();
                count++;
                rPWw_accepted.push_back(rPWw_ground[i]);
            }
        }
        if (count > 0) {
            new_scale_factor /= count;
        }

        // Exponential filter the scale factor
        scale_factor = (1 - cfg.stella_config.scale_factor_alpha) * scale_factor + cfg.stella_config.scale_factor_alpha * new_scale_factor;

        // Emit the stella map (like in original)
        auto stella_map = std::make_unique<StellaMap>();
        std::vector<Eigen::Vector3d> rPWw_map;
        for (size_t i = 0; i < rPCw_stella.size(); i++) {
            Eigen::Vector3d rPWw_scaled = Hwc.translation() + rPCw_stella[i] * scale_factor;
            rPWw_map.push_back(rPWw_scaled);
        }
        stella_map->rPWw_map = rPWw_map;
        emit(std::move(stella_map));

        // Adjust the Hnc transform to account for the scale factor
        auto Hnc = stella->Hnc;
        Hnc.translation() = Hnc.translation() * scale_factor;

        // Construct the Htw transform using the stella pose
        auto Htw_stella_raw = Htc * Hnc.inverse() * Hwn.inverse();

        // Apply constraints: Force Stella to use kinematic z + IMU roll/pitch
        auto Hwt_stella_unconstrained = Htw_stella_raw.inverse();  // Get world-to-torso transform from Stella
        Eigen::Vector3d rpy_imu = mat_to_rpy_intrinsic(Rwt_mahony);  // Get IMU roll/pitch from Mahony filter
        Eigen::Vector3d rpy_stella = mat_to_rpy_intrinsic(Hwt_stella_unconstrained.linear());  // Get Stella's original attitude

        // Create constrained transform: Stella's x,y,yaw + kinematic z + IMU roll/pitch
        Hwt_stella = Eigen::Isometry3d::Identity();

        // Keep Stella's x, y position
        Hwt_stella.translation().x() = Hwt_stella_unconstrained.translation().x();
        Hwt_stella.translation().y() = Hwt_stella_unconstrained.translation().y();

        // Apply constraints:
        // - z from kinematics: g_z(z) = z_stella - z_kinematic = 0
        Hwt_stella.translation().z() = Hwt_kinematic.translation().z();

        // - roll/pitch from IMU: g_roll(roll) = roll_stella - roll_imu = 0
        //                        g_pitch(pitch) = pitch_stella - pitch_imu = 0
        // - yaw from Stella (preserve Stella's yaw estimate)
        Eigen::Vector3d rpy_constrained(rpy_imu.x(),      // Roll from IMU
                                       rpy_imu.y(),      // Pitch from IMU
                                       rpy_stella.z());  // Yaw from Stella
        Hwt_stella.linear() = rpy_intrinsic_to_mat(rpy_constrained);

        // Set Hwn for sensors output
        sensors->Hwn = Hwn;

        // Emit debug graphs for monitoring (like in original)
        emit(graph("Roll IMU", rpy_imu.x()));
        emit(graph("Pitch IMU", rpy_imu.y()));
        emit(graph("Roll stella original", rpy_stella.x()));
        emit(graph("Pitch stella original", rpy_stella.y()));
        emit(graph("Roll constrained", rpy_constrained.x()));
        emit(graph("Pitch constrained", rpy_constrained.y()));
        emit(graph("Z anchor (kinematic)", Hwt_kinematic.translation().z()));
        emit(graph("Z stella original", Hwt_stella_unconstrained.translation().z()));
        emit(graph("Z constrained", Hwt_stella.translation().z()));
        emit(graph("Roll constraint error", rpy_stella.x() - rpy_imu.x()));
        emit(graph("Pitch constraint error", rpy_stella.y() - rpy_imu.y()));
        emit(graph("Z constraint error", Hwt_stella_unconstrained.translation().z() - Hwt_kinematic.translation().z()));

        stella_available = true;
    }

    // 3. Collect measurements for sliding window optimization
    std::vector<MeasurementFactor> current_factors;

    // Add kinematic measurement if healthy
    if (fault_status.kinematics_healthy) {
        MeasurementFactor kinematic_factor;
        kinematic_factor.type = MeasurementFactor::KINEMATIC;
        kinematic_factor.time_index = 0;  // Current time step
        kinematic_factor.measurement = Eigen::Vector2d(Hwt_kinematic.translation().x(),
                                                      Hwt_kinematic.translation().y());
        kinematic_factor.weight = cfg.sliding_window.kinematic_base_weight * fault_status.kinematics_confidence;
        current_factors.push_back(kinematic_factor);
        log<INFO>("Added kinematic measurement: [" + std::to_string(kinematic_factor.measurement.x()) +
                  ", " + std::to_string(kinematic_factor.measurement.y()) + "] weight: " +
                  std::to_string(kinematic_factor.weight));
    }

    // Add Stella measurement if available and healthy
    if (fault_status.stella_healthy) {
        MeasurementFactor stella_factor;
        stella_factor.type = MeasurementFactor::STELLA;
        stella_factor.time_index = 0;  // Current time step
        stella_factor.measurement = Eigen::Vector2d(Hwt_stella.translation().x(),
                                                   Hwt_stella.translation().y());
        stella_factor.weight = cfg.sliding_window.stella_base_weight * fault_status.stella_confidence;
        current_factors.push_back(stella_factor);
        log<INFO>("Added Stella measurement: [" + std::to_string(stella_factor.measurement.x()) +
                  ", " + std::to_string(stella_factor.measurement.y()) + "] weight: " +
                  std::to_string(stella_factor.weight));
    }

    // 4. Run sliding window optimization
    Eigen::Isometry3d Hwt_final;
    double optimization_cost = -1.0;  // Add this line to track cost outside the block

    if (!current_factors.empty()) {
        // Create initial guess (weighted average of available measurements)
        Eigen::Vector2d initial_pos = Eigen::Vector2d::Zero();
        double total_weight = 0.0;
        for (const auto& factor : current_factors) {
            initial_pos += factor.weight * factor.measurement;
            total_weight += factor.weight;
        }
        if (total_weight > 0) {
            initial_pos /= total_weight;
        }

        Eigen::VectorXd initial_guess(2);
        initial_guess << initial_pos.x(), initial_pos.y();

        log<INFO>("Running sliding window optimization with " + std::to_string(current_factors.size()) +
                  " factors (Stella healthy: " + std::to_string(fault_status.stella_healthy) +
                  ", Kinematic healthy: " + std::to_string(fault_status.kinematics_healthy) + ")");

        // Call your optimization!
        auto [optimized_states, cost] = optimize_sliding_window(initial_guess, current_factors);
        optimization_cost = cost;  // Store cost for later use

        log<INFO>("Optimization completed with cost: " + std::to_string(cost));

        // Use optimized result
        Hwt_final = Eigen::Isometry3d::Identity();
        Hwt_final.translation().x() = optimized_states[0];
        Hwt_final.translation().y() = optimized_states[1];
        Hwt_final.translation().z() = Hwt_kinematic.translation().z();  // Always use kinematic z

    } else {
        log<WARN>("No healthy measurements available for sliding window optimization - using kinematic fallback");
        Hwt_final = Hwt_kinematic;
    }

    // 5. Set final orientation (IMU roll/pitch + fused yaw)
    const double kinematic_yaw = mat_to_rpy_intrinsic(Hwt_kinematic.linear()).z();
    const double fused_yaw = yaw_filter.update(sensors->gyroscope.z(), kinematic_yaw, dt);
    Eigen::Vector3d rpy_final_orientation(rpy_mahony.x(), rpy_mahony.y(), fused_yaw);  // Rename this variable
    Hwt_final.linear() = rpy_intrinsic_to_mat(rpy_final_orientation);

    // 6. Set final sensor outputs
    sensors->Htw = Hwt_final.inverse();

    // Robot pose (x,y translation + yaw rotation only)
    Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
    Hwr.linear() = Eigen::AngleAxisd(fused_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Hwr.translation() = Eigen::Vector3d(Hwt_final.translation().x(), Hwt_final.translation().y(), 0.0);
    sensors->Hrw = Hwr.inverse();

    // Velocity estimation
    if (previous_sensors) {
        auto prev_pos = previous_sensors->Htw.inverse().translation();
        auto curr_pos = Hwt_final.translation();
        sensors->vTw = Eigen::Vector3d((curr_pos.x() - prev_pos.x()) / dt,
                                      (curr_pos.y() - prev_pos.y()) / dt,
                                      0.0);
    } else {
        sensors->vTw = Eigen::Vector3d::Zero();
    }

    // Debug output
    emit(graph("SW Optimization Cost", optimization_cost));  // Use the stored cost

    // Position outputs
    emit(graph("SW Position X", Hwt_final.translation().x()));
    emit(graph("SW Position Y", Hwt_final.translation().y()));
    emit(graph("SW Position Z", Hwt_final.translation().z()));

    // Orientation outputs - use the final orientation variable
    emit(graph("SW Roll", rpy_final_orientation.x()));
    emit(graph("SW Pitch", rpy_final_orientation.y()));
    emit(graph("SW Yaw", rpy_final_orientation.z()));

    // Health status
    emit(graph("SW Stella Health", fault_status.stella_confidence));
    emit(graph("SW Kinematic Health", fault_status.kinematics_confidence));
    emit(graph("SW Num Factors", static_cast<double>(current_factors.size())));

    // Compare with individual measurements
    if (fault_status.kinematics_healthy) {
        emit(graph("SW Kinematic X", Hwt_kinematic.translation().x()));
        emit(graph("SW Kinematic Y", Hwt_kinematic.translation().y()));
    }

    if (stella_available && fault_status.stella_healthy) {
        emit(graph("SW Stella X", Hwt_stella.translation().x()));
        emit(graph("SW Stella Y", Hwt_stella.translation().y()));
    }

    // Optimization vs measurements comparison
    if (!current_factors.empty()) {
        for (size_t i = 0; i < current_factors.size(); ++i) {
            const auto& factor = current_factors[i];
            std::string prefix = (factor.type == MeasurementFactor::STELLA) ? "SW Stella" : "SW Kinematic";
            emit(graph(prefix + " Meas X " + std::to_string(i), factor.measurement.x()));
            emit(graph(prefix + " Meas Y " + std::to_string(i), factor.measurement.y()));
            emit(graph(prefix + " Weight " + std::to_string(i), factor.weight));
        }
    }

}


}  // namespace module::input
