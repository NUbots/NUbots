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
#include <fmt/format.h>
#include <nuclear>

#include "extension/Behaviour.hpp"

#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"

#define TORQUE_ENABLED 100

namespace module::actuation {

    using message::actuation::ServoState;
    using message::input::Sensors;

    using utility::input::LimbID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;

    /// @brief Empty struct used to trigger the setting of servo gains.
    struct SetGains {};

    class FootController : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the FootController reactor.
        explicit FootController(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Foot controller mode
            std::string mode = "IK";

            /// @brief Map between ServoID and ServoState
            std::map<utility::input::ServoID, ServoState> servo_states = {
                {utility::input::ServoID::L_HIP_YAW, ServoState()},
                {utility::input::ServoID::L_HIP_ROLL, ServoState()},
                {utility::input::ServoID::L_HIP_PITCH, ServoState()},
                {utility::input::ServoID::L_KNEE, ServoState()},
                {utility::input::ServoID::L_ANKLE_PITCH, ServoState()},
                {utility::input::ServoID::L_ANKLE_ROLL, ServoState()},
                {utility::input::ServoID::R_HIP_YAW, ServoState()},
                {utility::input::ServoID::R_HIP_ROLL, ServoState()},
                {utility::input::ServoID::R_HIP_PITCH, ServoState()},
                {utility::input::ServoID::R_KNEE, ServoState()},
                {utility::input::ServoID::R_ANKLE_PITCH, ServoState()},
                {utility::input::ServoID::R_ANKLE_ROLL, ServoState()},
            };
            /// @brief Startup gain before setting the desired gains
            double startup_gain = 0;
            /// @brief Delay before setting the desired gains
            std::chrono::seconds set_gain_delay = std::chrono::seconds(0);
            /// @brief Desired gains for each servo
            std::map<std::string, double> desired_gains{};

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
            /// @brief Maximum allowed pitch error before disabling correction
            double max_pitch_error = 0.0;
            /// @brief Maximum allowed roll error before disabling correction
            double max_roll_error = 0.0;
        } cfg;

        /// @brief Accumulates the integral of the roll error over time for use in PID control.
        double integral_roll_error = 0;
        /// @brief Accumulates the integral of the pitch error over time for use in PID control.
        double integral_pitch_error = 0;

        /// @brief Stores the timestamp of the last PID control update, used to compute time deltas for integral and
        /// derivative calculations.
        NUClear::clock::time_point last_update_time{};

        /// @brief Foot controller logic
        template <typename FootControlTask, typename IKTask>
        void control_foot(const FootControlTask& foot_control_task,
                          IKTask& ik_task,
                          const Sensors& sensors,
                          LimbID limb_id) {

            ik_task->time = foot_control_task.time;
            ik_task->Htf  = foot_control_task.Htf;

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

            // Add correction to the torso orientation if enabled
            if (foot_control_task.correction_enabled) {
                // Hwt quaternion
                Eigen::Quaterniond Hwt_quat(sensors.Htw.inverse().linear());

                // Get fused roll and pitch
                Eigen::Vector3d rpy = mat_to_rpy_intrinsic(Hwt_quat.toRotationMatrix());
                double fused_roll   = rpy.x();
                double fused_pitch  = rpy.y();

                if (log_level <= DEBUG) {
                    // Graph the correction being applied
                    emit(graph("Balance/Actual_Roll", fused_roll));
                    emit(graph("Balance/Actual_Pitch", fused_pitch));
                }

                // Get the desired roll and pitch
                Eigen::Quaterniond Hft_quat(ik_task->Htf.inverse().linear());
                rpy                  = mat_to_rpy_intrinsic(Hft_quat.toRotationMatrix());
                double desired_roll  = rpy.x();
                double desired_pitch = rpy.y();

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
                auto clamped_roll_error  = utility::math::clamp(-cfg.max_roll_error, roll_error, cfg.max_roll_error);
                auto clamped_pitch_error = utility::math::clamp(-cfg.max_pitch_error, pitch_error, cfg.max_pitch_error);
                desired_roll += cfg.roll_p_gain * clamped_roll_error;
                desired_pitch += cfg.pitch_p_gain * clamped_pitch_error;

                if (log_level <= DEBUG) {
                    if (roll_error > cfg.max_roll_error || roll_error < -cfg.max_roll_error) {
                        // If the error is too large, we are probably falling over and should pause applying control.
                        log<DEBUG>(fmt::format("Balance correction disabled due to large ROLL error: roll_error = {}",
                                               roll_error));
                        log<DEBUG>(fmt::format("Clamping roll error to: {}", clamped_roll_error));
                    }

                    if (pitch_error > cfg.max_pitch_error || pitch_error < -cfg.max_pitch_error) {
                        // If the error is too large, we are probably falling over and should pause applying control.
                        log<DEBUG>(fmt::format("Balance correction disabled due to large error: pitch_error = {}",
                                               pitch_error));
                        log<DEBUG>(fmt::format("Clamping pitch error to: {}", clamped_pitch_error));
                    }
                }

                // Graph P Correction
                if (log_level <= DEBUG) {
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

                // D control using gyroscope values
                auto roll_rate  = sensors.gyroscope.x();
                auto pitch_rate = sensors.gyroscope.y();
                desired_roll -= cfg.roll_d_gain * roll_rate;
                desired_pitch -= cfg.pitch_d_gain * pitch_rate;

                if (log_level <= DEBUG) {
                    // Graph D corrections and rates
                    emit(graph("Balance/D_Roll_Correction", -cfg.roll_d_gain * roll_rate));
                    emit(graph("Balance/D_Pitch_Correction", -cfg.pitch_d_gain * pitch_rate));
                    emit(graph("Balance/Roll_Rate", roll_rate));
                    emit(graph("Balance/Pitch_Rate", pitch_rate));

                    // Graph final corrected desired values
                    emit(graph("Balance/Desired_Roll_Final", desired_roll));
                    emit(graph("Balance/Desired_Pitch_Final", desired_pitch));
                }

                double desired_yaw = mat_to_rpy_intrinsic(Hft_quat.toRotationMatrix()).z();

                // Compute desired orientation: yaw * fused_roll_pitch
                Eigen::Matrix3d desired_Rft = Eigen::AngleAxisd(desired_yaw, Eigen::Vector3d::UnitZ())
                                              * rpy_intrinsic_to_mat(Eigen::Vector3d(desired_roll, desired_pitch, 0.0));
                Eigen::Isometry3d Htf_corrected = foot_control_task.Htf;
                Htf_corrected.linear()          = desired_Rft.transpose();

                ik_task->Htf = Htf_corrected;
            }
        }
    };


}  // namespace module::actuation

#endif  // MODULE_ACTUATION_FOOTCONTROLLER_HPP
