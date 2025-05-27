/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#ifndef MODULE_ACTUATION_FOOTCONTROLLER_HPP
#define MODULE_ACTUATION_FOOTCONTROLLER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "extension/Behaviour.hpp"

#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/ControlFoot.hpp"

#include "utility/file/fileutil.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"

#define TORQUE_ENABLED 100

namespace module::actuation {

    using message::actuation::LeftLegIK;
    using message::actuation::RightLegIK;
    using message::actuation::ServoState;
    using message::input::Sensors;
    using message::skill::ControlLeftFoot;
    using message::skill::ControlRightFoot;

    using utility::input::LimbID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;

    struct SetGains {};

    // Function declarations
    void FusedFromQuat(const Eigen::Quaterniond& q, double& fusedPitch, double& fusedRoll);
    Eigen::Quaterniond QuatFromFused(double fusedPitch, double fusedRoll);

    class FootController : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Foot controller mode
            std::string mode = "IK";

            /// @brief Map between ServoID and ServoState
            std::map<utility::input::ServoID, message::actuation::ServoState> servo_states = {
                {utility::input::ServoID::L_HIP_YAW, message::actuation::ServoState()},
                {utility::input::ServoID::L_HIP_ROLL, message::actuation::ServoState()},
                {utility::input::ServoID::L_HIP_PITCH, message::actuation::ServoState()},
                {utility::input::ServoID::L_KNEE, message::actuation::ServoState()},
                {utility::input::ServoID::L_ANKLE_PITCH, message::actuation::ServoState()},
                {utility::input::ServoID::L_ANKLE_ROLL, message::actuation::ServoState()},
                {utility::input::ServoID::R_HIP_YAW, message::actuation::ServoState()},
                {utility::input::ServoID::R_HIP_ROLL, message::actuation::ServoState()},
                {utility::input::ServoID::R_HIP_PITCH, message::actuation::ServoState()},
                {utility::input::ServoID::R_KNEE, message::actuation::ServoState()},
                {utility::input::ServoID::R_ANKLE_PITCH, message::actuation::ServoState()},
                {utility::input::ServoID::R_ANKLE_ROLL, message::actuation::ServoState()},
            };
            /// @brief Startup gain before setting the desired gains
            double startup_gain = 0;
            /// @brief Delay before setting the desired gains
            std::chrono::seconds set_gain_delay = std::chrono::seconds(0);
            /// @brief Desired gains for each servo
            std::map<std::string, double> desired_gains{};

            /// @brief Whether or not to use orientation correction
            bool correction_enabled = false;
            /// @brief Proportional gain for torso orientation roll correction
            double roll_p_gain = 0.0;
            /// @brief Proportional gain for torso orientation pitch correction
            double pitch_p_gain = 0.0;
            /// @brief Integral gain for torso orientation roll correction
            double roll_i_gain = 0.0;
            /// @brief Integral gain for torso orientation pitch correction
            double pitch_i_gain = 0.0;
            /// @brief Antiwindup max I error
            double max_i_error = 0.0;
            /// @brief Derivative gain for torso orientation roll correction
            double roll_d_gain = 0.0;
            /// @brief Derivative gain for torso orientation pitch correction
            double pitch_d_gain = 0.0;
            /// @brief offsets for support foot when kicking
            double support_foot_offset_x = 0.0;
            double support_foot_offset_y = 0.0;
            double support_foot_offset_z = 0.0;
        } cfg;

        // *************** //
        // *** Legs *** //
        // *************** //
        double prev_roll_error      = 0;
        double prev_pitch_error     = 0;
        double integral_roll_error  = 0;
        double integral_pitch_error = 0;
        /// @brief Last time we updated
        NUClear::clock::time_point last_update_time{};


    public:
        /// @brief Called by the powerplant to build and setup the FootController reactor.
        explicit FootController(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Foot controller logic
        template <typename FootControlTask, typename IKTask>
        void control_foot(const FootControlTask& foot_control_task,
                          IKTask& ik_task,
                          const Sensors& sensors,
                          LimbID limb_id) {

            ik_task->time = foot_control_task.time;
            ik_task->Htf  = foot_control_task.Htf;

            if (foot_control_task.correction_enabled && cfg.correction_enabled) {
                // Hwt quaternion
                Eigen::Quaterniond Hwt_quat(sensors.Htw.inverse().linear());
                // Test before use
                // emit(graph("Debug/Hwt_quat_w_before_use", Hwt_quat.w()));
                // emit(graph("Debug/Hwt_quat_x_before_use", Hwt_quat.x()));
                // emit(graph("Debug/Hwt_quat_y_before_use", Hwt_quat.y()));
                // emit(graph("Debug/Hwt_quat_z_before_use", Hwt_quat.z()));

                // Get fused roll and pitch
                double fused_roll;
                double fused_pitch;
                FusedFromQuat(Hwt_quat, fused_pitch, fused_roll);

                if (log_level <= DEBUG) {
                    // graph the correction being applied
                    emit(graph("Balance/Actual_Roll", fused_roll));
                    emit(graph("Balance/Actual_Pitch", fused_pitch));
                }

                // Add offset from the config.
                // NOTE: This is a hack to counter a backwards tilt
                // that i don't know where it comes from.
                // Update: It was coming from the uninitialised Hft Quaternion.
                // TODO: Test if this occurs with the balancer disabled
                fused_pitch += cfg.support_foot_offset_x;
                fused_roll += cfg.support_foot_offset_y;

                // Get the desired roll and pitch
                Eigen::Quaterniond Hft_quat;
                Hft_quat = ik_task->Htf.inverse().linear();

                // Eigen::Quaterniond Hft_quat(ik_task->Htf.inverse().linear());
                // Debug quat before use
                // emit(graph("Debug/Hft_quat_w_before_use", Hft_quat.w()));
                // emit(graph("Debug/Hft_quat_x_before_use", Hft_quat.x()));
                // emit(graph("Debug/Hft_quat_y_before_use", Hft_quat.y()));
                // emit(graph("Debug/Hft_quat_z_before_use", Hft_quat.z()));

                double desired_roll;
                double desired_pitch;
                FusedFromQuat(Hft_quat, desired_pitch, desired_roll);

                if (log_level <= DEBUG) {
                    // Graph desired orientation (before PID correction)
                    emit(graph("Balance/Desired_Roll_Original", desired_roll));
                    emit(graph("Balance/Desired_Pitch_Original", desired_pitch));
                }

                // Compute the error between the desired torso orientation and the actual torso orientation
                auto roll_error  = desired_roll - fused_roll;
                auto pitch_error = desired_pitch - fused_pitch;

                if (log_level <= DEBUG) {
                    // Graph the errors
                    emit(graph("Balance/Roll_Error", roll_error));
                    emit(graph("Balance/Pitch_Error", pitch_error));
                }

                auto dt =
                    std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - last_update_time)
                        .count();
                last_update_time = NUClear::clock::now();

                // P control
                desired_roll += cfg.roll_p_gain * roll_error;
                desired_pitch += cfg.pitch_p_gain * pitch_error;

                // Graph P Correction
                if (log_level <= DEBUG) {
                    // Graph the desired roll and pitch after P control
                    emit(graph("Balance/P_Roll_Correction", cfg.roll_p_gain * roll_error));
                    emit(graph("Balance/P_Pitch_Correction", cfg.pitch_p_gain * pitch_error));
                }

                // I control
                integral_roll_error += roll_error * dt;
                integral_pitch_error += pitch_error * dt;

                // Graph the integral correction
                if (log_level <= DEBUG) {
                    emit(graph("Balance/I_Roll_Correction", cfg.roll_i_gain * integral_roll_error));
                    emit(graph("Balance/I_Pitch_Correction", cfg.pitch_i_gain * integral_pitch_error));
                }

                // Anti windup
                integral_roll_error  = std::max(std::min(integral_roll_error, cfg.max_i_error), -cfg.max_i_error);
                integral_pitch_error = std::max(std::min(integral_pitch_error, cfg.max_i_error), -cfg.max_i_error);

                desired_roll += cfg.roll_i_gain * integral_roll_error;
                desired_pitch += cfg.pitch_i_gain * integral_pitch_error;

                // D control
                // NOTE: Can get the values from the gyroscope
                // *****Original implementation****
                // auto roll_error_rate  = (roll_error - prev_roll_error) / dt;
                // auto pitch_error_rate = (pitch_error - prev_pitch_error) / dt;
                // prev_roll_error       = roll_error;
                // prev_pitch_error      = pitch_error;

                // desired_roll += cfg.roll_d_gain * roll_error_rate;
                // desired_pitch += cfg.pitch_d_gain * pitch_error_rate;
                // ********************************

                // *****Using gyroscope values*****
                auto roll_rate  = sensors.gyroscope.x();
                auto pitch_rate = sensors.gyroscope.y();
                desired_roll -= cfg.roll_d_gain * roll_rate;
                desired_pitch -= cfg.pitch_d_gain * pitch_rate;
                // ********************************

                if (log_level <= DEBUG) {
                    // Graph D corrections and rates
                    emit(graph("Balance/D_Roll_Correction", -cfg.roll_d_gain * roll_rate));
                    emit(graph("Balance/D_Pitch_Correction", -cfg.pitch_d_gain * pitch_rate));
                    emit(graph("Balance/Roll_Rate", roll_rate));
                    emit(graph("Balance/Pitch_Rate", pitch_rate));

                    // Graph final corrected desired values
                    emit(graph("Balance/Desired_Roll_Final", desired_roll));
                    emit(graph("Balance/Desired_Pitch_Final", desired_pitch));

                    // Graph the raw sensor values
                    emit(graph("Debug/Raw_Accel_X", sensors.accelerometer.x()));
                    emit(graph("Debug/Raw_Accel_Y", sensors.accelerometer.y()));
                    emit(graph("Debug/Raw_Accel_Z", sensors.accelerometer.z()));
                }

                double desired_yaw = mat_to_rpy_intrinsic(Hft_quat.toRotationMatrix()).z();

                // Compute desired orientation: yaw * fused_roll_pitch
                Eigen::Matrix3d desired_Rft = Eigen::AngleAxisd(desired_yaw, Eigen::Vector3d::UnitZ())
                                              * QuatFromFused(desired_pitch, desired_roll).toRotationMatrix();
                Eigen::Isometry3d Htf_corrected = foot_control_task.Htf;
                Htf_corrected.linear()          = desired_Rft.transpose();

                ik_task->Htf = Htf_corrected;
            }

            else {
                ik_task->Htf = foot_control_task.Htf;
            }

            if (cfg.mode == "IK") {
                for (auto id : utility::input::LimbID::servos_for_limb(limb_id)) {
                    ik_task->servos[id] = ServoState(cfg.servo_states[id].gain, TORQUE_ENABLED);
                    // Get servo from sensors for graphing information
                    auto it          = std::find_if(sensors.servo.begin(),
                                           sensors.servo.end(),
                                           [id](const message::input::Sensors::Servo& servo) {
                                               return servo.id == static_cast<uint32_t>(id);
                                           });
                    auto servo       = *it;
                    std::string name = static_cast<std::string>(id);
                    emit(graph("Servo Present Position/" + name, servo.present_position));
                    emit(graph("Servo Goal Position/" + name, servo.goal_position));
                    emit(graph("Servo Error/" + name, servo.present_position - servo.goal_position));
                }
            }
            else {
                throw std::runtime_error("Invalid mode");
            }
        }
    };

    // Conversion: Quaternion --> Fused angles (2D)
    void FusedFromQuat(const Eigen::Quaterniond& q, double& fusedPitch, double& fusedRoll) {
        // Calculate the fused pitch and roll
        double stheta = 2.0 * (q.y() * q.w() - q.x() * q.z());
        double sphi   = 2.0 * (q.y() * q.z() + q.x() * q.w());
        stheta        = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta));  // Coerce stheta to [-1,1]
        sphi          = (sphi >= 1.0 ? 1.0 : (sphi <= -1.0 ? -1.0 : sphi));        // Coerce sphi   to [-1,1]
        fusedPitch    = asin(stheta);
        fusedRoll     = asin(sphi);
    }

    Eigen::Quaterniond QuatFromFused(double fusedPitch, double fusedRoll)  // Assume: fusedYaw = 0, hemi = true
    {
        // Precalculate the sine values
        double sth  = sin(fusedPitch);
        double sphi = sin(fusedRoll);

        // Calculate the sine sum criterion
        double crit = sth * sth + sphi * sphi;

        // Calculate the tilt angle alpha
        double alpha   = (crit >= 1.0 ? M_PI_2 : acos(sqrt(1.0 - crit)));
        double halpha  = 0.5 * alpha;
        double chalpha = cos(halpha);
        double shalpha = sin(halpha);

        // Calculate the tilt axis angle gamma
        double gamma  = atan2(sth, sphi);
        double cgamma = cos(gamma);
        double sgamma = sin(gamma);

        // Return the required quaternion orientation (a rotation about (cgamma, sgamma, 0) by angle alpha)
        Eigen::Quaterniond result = Eigen::Quaterniond(chalpha, cgamma * shalpha, sgamma * shalpha, 0.0);
        return result;  // Order: (w,x,y,z)
    }


}  // namespace module::actuation

#endif  // MODULE_ACTUATION_FOOTCONTROLLER_HPP
