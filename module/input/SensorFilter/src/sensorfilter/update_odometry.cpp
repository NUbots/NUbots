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
#include <chrono>

#include "SensorFilter.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/math/euler.hpp"

namespace module::input {

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::platform::RawSensors;

    using utility::input::FrameID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;

    void SensorFilter::update_odometry(std::unique_ptr<Sensors>& sensors,
                                       const std::shared_ptr<const Sensors>& previous_sensors,
                                       const RawSensors& raw_sensors,
                                       const Stability& stability) {
        // Use ground truth instead of calculating odometry, then return
        if (cfg.use_ground_truth) {
            // Construct world {w} to torso {t} space transform from ground truth.inverse();
            sensors->Htw = Eigen::Isometry3d(raw_sensors.odometry_ground_truth.Htw);
            // Construct robot {r} to world {w} space transform from ground truth
            Eigen::Isometry3d Hrw = Eigen::Isometry3d::Identity();
            Hrw.linear() = Eigen::AngleAxisd(mat_to_rpy_intrinsic(sensors->Htw.linear()).z(), Eigen::Vector3d::UnitZ())
                               .toRotationMatrix();
            Hrw.translation() = Eigen::Vector3d(sensors->Htw.translation().x(), sensors->Htw.translation().y(), 0.0);
            sensors->Hrw      = Hrw;
            sensors->vTw      = raw_sensors.odometry_ground_truth.vTw;
            return;
        }

        // Compute time since last update
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // Apply learned bias correction to gyroscope before filter update
        Eigen::Vector3d corrected_gyro = sensors->gyroscope - learned_gyro_bias;

        // Perform Madgwick update with bias-corrected gyroscope
        madgwick_filter.update(corrected_gyro, sensors->accelerometer, dt);

        // Adaptive bias learning (only after initial settling period)
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_start =
            std::chrono::duration_cast<std::chrono::duration<double>>(current_time - filter_start_time).count();

        if (time_since_start > bias_learning_delay) {
            // Log when bias learning first starts
            static bool learning_started = false;
            if (!learning_started) {
                learning_started = true;
                log<INFO>("Starting adaptive gyro bias learning. Current learned bias: ",
                          learned_gyro_bias.transpose());
            }

            // Get current filter bias estimate and slowly adapt our learned bias
            Eigen::Vector3d filter_bias = madgwick_filter.get_gyro_bias();
            learned_gyro_bias += bias_learning_rate * filter_bias * dt;

            // Clamp learned bias to reasonable values (prevent runaway)
            learned_gyro_bias = learned_gyro_bias.cwiseMax(-0.2).cwiseMin(0.2);  // ±0.2 rad/s max bias
        }

        // Orientation stability check - detect if filter has flipped or is unstable
        auto current_orientation       = madgwick_filter.get_quaternion();
        double orientation_change      = std::abs(current_orientation.angularDistance(prev_orientation));
        double orientation_change_rate = dt > 0 ? orientation_change / dt : 0.0;

        if (orientation_change_rate > max_orientation_change_rate && previous_sensors != nullptr) {
            log<WARN>("Orientation instability detected (", orientation_change_rate, " rad/s), correcting filter");

            // Don't reset completely - just nudge back toward stable state
            // Use previous orientation as reference and blend
            Eigen::Quaterniond stable_orientation = prev_orientation.slerp(0.1, current_orientation);
            madgwick_filter.set_quaternion(stable_orientation);
        }

        // Store current orientation for next iteration
        prev_orientation = madgwick_filter.get_quaternion();

        // Debug logging for bias learning (every 100 iterations to avoid spam)
        static int debug_counter = 0;
        if (++debug_counter % 100 == 0 && log_level <= DEBUG) {
            log<DEBUG>("Learned bias: ",
                       learned_gyro_bias.transpose(),
                       " | Filter bias: ",
                       madgwick_filter.get_gyro_bias().transpose(),
                       " | Time since start: ",
                       time_since_start,
                       "s");
        }
        const auto Rwt_madgwick      = madgwick_filter.get_rotation_matrix();
        Eigen::Vector3d rpy_madgwick = mat_to_rpy_intrinsic(Rwt_madgwick);

        // If fallen, keep position still
        if (stability <= Stability::FALLING) {
            Eigen::Isometry3d Hwt =
                previous_sensors == nullptr ? Eigen::Isometry3d::Identity() : previous_sensors->Htw.inverse();
            Hwt.linear() = Rwt_madgwick;
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

        // Construct world {w} to torso {t} space transform (madgwick orientation, anchor translation)
        Eigen::Isometry3d Hwt = Eigen::Isometry3d::Identity();
        Hwt.translation()     = Hwt_anchor.translation();
        Hwt.linear()          = Rwt_madgwick;
        sensors->Htw          = Hwt.inverse();

        // Construct robot {r} to world {w} space transform (just x-y translation and yaw rotation)
        Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
        Hwr.linear()          = Eigen::AngleAxisd(rpy_madgwick.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Hwr.translation()     = Eigen::Vector3d(Hwt_anchor.translation().x(), Hwt_anchor.translation().y(), 0.0);
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

}  // namespace module::input
