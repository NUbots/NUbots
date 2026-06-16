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

namespace module::input {

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::platform::RawSensors;

    using utility::input::FrameID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;

    void SensorFilter::update_odometry(std::unique_ptr<Sensors>& sensors,
                                       const std::shared_ptr<const Sensors>& previous_sensors,
                                       const RawSensors& raw_sensors,
                                       const message::behaviour::state::Stability& stability,
                                       const std::shared_ptr<const RobotPoseGroundTruth>& robot_pose_ground_truth) {
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

            // Store history to remain continuous
            position_history.push_back(Hwt.translation());
            rotation_history.push_back(Hwt.linear());
            if (position_history.size() > WINDOW_SIZE) {
                position_history.pop_front();
            }
            if (rotation_history.size() > WINDOW_SIZE) {
                rotation_history.pop_front();
            }

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

        // Combine Mahony roll/pitch with fused yaw
        Eigen::Vector3d rpy_fused(rpy_mahony.x(), rpy_mahony.y(), fused_yaw);
        Hwt.linear() = rpy_intrinsic_to_mat(rpy_fused);

        // Gather features for neural network
        std::vector<float> fv;
        fv.reserve(FEATURE_DIM);

        fv.push_back(static_cast<float>(last_walk_state.velocity_target.x()));
        fv.push_back(static_cast<float>(last_walk_state.velocity_target.y()));
        fv.push_back(static_cast<float>(last_walk_state.velocity_target.z()));

        fv.push_back(static_cast<float>(sensors->accelerometer.x()));
        fv.push_back(static_cast<float>(sensors->accelerometer.y()));
        fv.push_back(static_cast<float>(sensors->accelerometer.z()));

        fv.push_back(static_cast<float>(sensors->gyroscope.x()));
        fv.push_back(static_cast<float>(sensors->gyroscope.y()));
        fv.push_back(static_cast<float>(sensors->gyroscope.z()));

        for (const auto& servo : sensors->servo) {
            fv.push_back(static_cast<float>(servo.present_position));
        }
        for (const auto& servo : sensors->servo) {
            fv.push_back(static_cast<float>(servo.present_velocity));
        }
        for (const auto& servo : sensors->servo) {
            fv.push_back(static_cast<float>(servo.goal_position));
        }

        feature_history.push_back(fv);
        if (feature_history.size() > WINDOW_SIZE) {
            feature_history.pop_front();
        }

        bool run_neural = cfg.neural_odom.use_neural_odometry && model_loaded
                          && (feature_history.size() == WINDOW_SIZE);

        if (run_neural) {
            try {
                std::vector<float> input_data;
                input_data.reserve(WINDOW_SIZE * FEATURE_DIM);
                for (const auto& f : feature_history) {
                    input_data.insert(input_data.end(), f.begin(), f.end());
                }

                auto input_port = compiled_model.input();
                ov::Shape static_shape = {1, size_t(WINDOW_SIZE * FEATURE_DIM)};
                ov::Tensor input_tensor(input_port.get_element_type(), static_shape);
                std::memcpy(input_tensor.data<float>(), input_data.data(), WINDOW_SIZE * FEATURE_DIM * sizeof(float));
                infer_request.set_input_tensor(input_tensor);

                infer_request.infer();
                auto output           = infer_request.get_output_tensor(0);
                const float* out_data = output.data<const float>();
                // Model outputs displacement in the training ground-truth frame, which is negated
                // relative to the integration convention used here (forward = +dx at runtime).
                double dx             = -out_data[0];
                double dy             = -out_data[1];
                double dtheta         = out_data[2];

                // If command velocity is near zero, suppress odometry updates to prevent drift when standing still
                if (last_walk_state.velocity_target.norm() < cfg.neural_odom.zero_velocity_threshold) {
                    dx     = 0.0;
                    dy     = 0.0;
                    dtheta = 0.0;
                }

                // Integrate step-by-step using previous frame's orientation and position
                Eigen::Vector3d p_prev = previous_sensors ? Eigen::Vector3d(previous_sensors->Htw.inverse().translation()) : Eigen::Vector3d(Hwt_anchor.translation());
                Eigen::Matrix3d R_prev = previous_sensors ? Eigen::Matrix3d(previous_sensors->Htw.inverse().linear()) : Eigen::Matrix3d(Hwt_anchor.linear());
                
                // Get previous integrated yaw
                double yaw_prev = previous_sensors ? mat_to_rpy_intrinsic(R_prev).z() : kinematic_yaw;
                
                // Create a yaw-only rotation matrix for the 2D local frame (heading frame)
                Eigen::Matrix3d R_yaw_prev = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, yaw_prev));
                
                // Rotate local displacement step (dx, dy) into world coordinates using previous yaw
                Eigen::Vector3d disp_world = R_yaw_prev * Eigen::Vector3d(dx, dy, 0.0);

                Hwt.translation() = p_prev + disp_world;
                Hwt.translation().z() = Hwt_anchor.translation().z();

                // Integrate neural yaw step update
                double yaw_curr = yaw_prev + dtheta;

                // Keep roll and pitch from Mahony filter, override yaw with neural yaw
                Eigen::Vector3d rpy_curr(rpy_mahony.x(), rpy_mahony.y(), yaw_curr);
                Hwt.linear() = rpy_intrinsic_to_mat(rpy_curr);

                // Construct robot {r} to world {w} using neural yaw so Hrw stays consistent with Htw
                Eigen::Isometry3d Hwr_neural = Eigen::Isometry3d::Identity();
                Hwr_neural.linear() = Eigen::AngleAxisd(yaw_curr, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                Hwr_neural.translation() = Eigen::Vector3d(Hwt.translation().x(), Hwt.translation().y(), 0.0);
                sensors->Hrw = Hwr_neural.inverse();
            }
            catch (const std::exception& e) {
                log<ERROR>("Neural Odometry inference failed: ", e.what());
                Hwt.translation() = Hwt_anchor.translation();
            }
        }
        else {
            Hwt.translation() = Hwt_anchor.translation();
        }

        // Store history for integration
        position_history.push_back(Hwt.translation());
        rotation_history.push_back(Hwt.linear());
        if (position_history.size() > WINDOW_SIZE) {
            position_history.pop_front();
        }
        if (rotation_history.size() > WINDOW_SIZE) {
            rotation_history.pop_front();
        }

        sensors->Htw = Hwt.inverse();

        // Construct robot {r} to world {w} space transform (just x-y translation and fused yaw rotation)
        // Only reached when neural odom did not run; otherwise Hrw is set inside the neural block above.
        if (!run_neural) {
            Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
            Hwr.linear()          = Eigen::AngleAxisd(fused_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Hwr.translation()     = Eigen::Vector3d(Hwt.translation().x(), Hwt.translation().y(), 0.0);
            sensors->Hrw          = Hwr.inverse();
        }

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
