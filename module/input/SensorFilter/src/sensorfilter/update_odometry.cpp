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

        // Estimate and correct the z-bias of the stella pose using kinematics
        if (z_bias_filter_initialized && dt > 0) {
            // Get z positions from both methods
            double z_anchor = Hwt_anchor.translation().z();  // Anchor point method z
            double z_stella = Htw_stella.inverse().translation().z();  // Stella method z

            // Calculate the observed z error (stella - anchor)
            double z_error = z_stella - z_anchor;

            // Time update (predict step) - no control input
            Eigen::Matrix<double, 0, 1> control_input;
            z_bias_filter.time(control_input, dt);

            // Measurement update (correct step) - measure the error
            Eigen::Matrix<double, 1, 1> measurement;
            measurement << z_error;
            z_bias_filter.measure(measurement);

            // Get the estimated bias
            double estimated_z_bias = z_bias_filter.get_state()(0);

            // Apply the bias correction to Hwn
            Eigen::Isometry3d Hwn_corrected = Hwn;
            Hwn_corrected.translation().z() -= estimated_z_bias;

            // Correct the Stella transform with the corrected Hwn
            auto Htw_stella_corrected = Htc * Hnc.inverse() * Hwn_corrected.inverse();
            sensors->Htw = Htw_stella_corrected;

            emit(graph("Z-bias estimate", estimated_z_bias));
            emit(graph("Z-error measurement", z_error));
            emit(graph("Z anchor", z_anchor));
            emit(graph("Z stella original", z_stella));
            emit(graph("Z stella corrected", Htw_stella_corrected.inverse().translation().z()));
        } else {
            // Use uncorrected Stella transform if filter not ready
            sensors->Htw = Htw_stella;
        }
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


}  // namespace module::input
