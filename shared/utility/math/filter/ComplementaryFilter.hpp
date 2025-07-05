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

#ifndef UTILITY_MATH_FILTER_COMPLEMENTARYFILTER_HPP
#define UTILITY_MATH_FILTER_COMPLEMENTARYFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>

namespace utility {
    namespace math {
        namespace filter {

            template <typename Scalar>
            class ComplementaryFilter {
            private:
                /// @brief Complementary filter coefficient (0-1)
                /// Higher values give more weight to the measurement, lower values to the prediction
                Scalar alpha = 0.1;

                /// @brief Bias learning rate (0-1)
                /// Higher values allow faster bias adaptation, lower values provide more stability
                Scalar beta = 0.01;

                /// @brief Current yaw estimate
                Scalar yaw = 0.0;

                /// @brief Estimated gyroscope bias
                Scalar bias = 0.0;

                /// @brief Whether the filter has been initialized
                bool initialized = false;

                /// @brief Previous gyroscope measurement for detecting stationary periods
                Scalar prev_gyro = 0.0;

                /// @brief Stationary detection threshold
                Scalar stationary_threshold = 0.01;

                /// @brief Counter for stationary measurements
                int stationary_count = 0;

                /// @brief Required consecutive stationary measurements for bias update
                int min_stationary_count = 10;

            public:
                // Constructor
                ComplementaryFilter(const Scalar alpha = 0.1, const Scalar beta = 0.01) : alpha(alpha), beta(beta) {}

                /// @brief Update the filter with gyroscope measurement and kinematic measurement
                /// @param gyro_z Z-axis gyroscope measurement (yaw rate)
                /// @param kinematic_yaw Yaw measurement from kinematics
                /// @param dt Time step
                /// @return Fused yaw estimate
                Scalar update(const Scalar gyro_z, const Scalar kinematic_yaw, const Scalar dt) {
                    if (!initialized) {
                        yaw         = kinematic_yaw;
                        bias        = 0.0;
                        prev_gyro   = gyro_z;
                        initialized = true;
                        return yaw;
                    }

                    // Detect if robot is stationary (gyroscope reading is close to zero)
                    bool is_stationary = std::abs(gyro_z) < stationary_threshold;

                    if (is_stationary) {
                        stationary_count++;
                    }
                    else {
                        stationary_count = 0;
                    }

                    // Remove bias from gyroscope measurement
                    Scalar corrected_gyro = gyro_z - bias;

                    // Predict yaw using bias-corrected gyroscope integration
                    Scalar predicted_yaw = yaw + corrected_gyro * dt;

                    // Normalize angles to [-pi, pi]
                    predicted_yaw                   = normalize_angle(predicted_yaw);
                    Scalar normalized_kinematic_yaw = normalize_angle(kinematic_yaw);

                    // Handle angle wrapping for the difference
                    Scalar angle_diff = normalize_angle(normalized_kinematic_yaw - predicted_yaw);

                    // Complementary filter update
                    yaw = predicted_yaw + alpha * angle_diff;
                    yaw = normalize_angle(yaw);

                    // Update bias estimate only when robot is stationary for sufficient time
                    // and the kinematic measurement is stable
                    if (stationary_count >= min_stationary_count && std::abs(angle_diff) < 0.05) {
                        // When stationary, any persistent gyroscope reading is likely bias
                        bias += beta * gyro_z;
                    }

                    prev_gyro = gyro_z;
                    return yaw;
                }

                /// @brief Update the filter with only gyroscope measurement (no kinematic measurement available)
                /// @param gyro_z Z-axis gyroscope measurement (yaw rate)
                /// @param dt Time step
                /// @return Predicted yaw estimate
                Scalar update_gyro_only(const Scalar gyro_z, const Scalar dt) {
                    if (!initialized) {
                        return 0.0;
                    }

                    // Detect if robot is stationary (gyroscope reading is close to zero)
                    bool is_stationary = std::abs(gyro_z) < stationary_threshold;

                    if (is_stationary) {
                        stationary_count++;
                    }
                    else {
                        stationary_count = 0;
                    }

                    // Remove bias from gyroscope measurement
                    Scalar corrected_gyro = gyro_z - bias;

                    // Predict yaw using bias-corrected gyroscope integration only
                    yaw = yaw + corrected_gyro * dt;
                    yaw = normalize_angle(yaw);

                    // Update bias estimate when stationary (no kinematic measurement available)
                    if (stationary_count >= min_stationary_count) {
                        // When stationary, any persistent gyroscope reading is likely bias
                        bias += beta * gyro_z;
                    }

                    prev_gyro = gyro_z;
                    return yaw;
                }

                /// @brief Get the current yaw estimate
                Scalar get_yaw() const {
                    return yaw;
                }

                /// @brief Get the current bias estimate
                Scalar get_bias() const {
                    return bias;
                }

                /// @brief Set the current yaw estimate
                void set_yaw(const Scalar new_yaw) {
                    yaw         = normalize_angle(new_yaw);
                    initialized = true;
                }

                /// @brief Set the current bias estimate
                void set_bias(const Scalar new_bias) {
                    bias = new_bias;
                }

                /// @brief Set the complementary filter coefficient
                void set_alpha(const Scalar new_alpha) {
                    alpha = std::clamp(new_alpha, Scalar(0.0), Scalar(1.0));
                }

                /// @brief Set the bias learning rate
                void set_beta(const Scalar new_beta) {
                    beta = std::clamp(new_beta, Scalar(0.0), Scalar(1.0));
                }

                /// @brief Get the complementary filter coefficient
                Scalar get_alpha() const {
                    return alpha;
                }

                /// @brief Get the bias learning rate
                Scalar get_beta() const {
                    return beta;
                }

                /// @brief Set the stationary detection threshold
                void set_stationary_threshold(const Scalar threshold) {
                    stationary_threshold = threshold;
                }

                /// @brief Get the stationary detection threshold
                Scalar get_stationary_threshold() const {
                    return stationary_threshold;
                }

                /// @brief Set the minimum stationary count for bias updates
                void set_min_stationary_count(const int count) {
                    min_stationary_count = count;
                }

                /// @brief Get the minimum stationary count for bias updates
                int get_min_stationary_count() const {
                    return min_stationary_count;
                }

                /// @brief Get the current stationary count
                int get_stationary_count() const {
                    return stationary_count;
                }

                /// @brief Check if the robot is currently considered stationary
                bool is_stationary() const {
                    return stationary_count >= min_stationary_count;
                }

                /// @brief Reset the filter
                void reset() {
                    yaw              = 0.0;
                    bias             = 0.0;
                    prev_gyro        = 0.0;
                    stationary_count = 0;
                    initialized      = false;
                }

            private:
                /// @brief Normalize angle to [-pi, pi]
                static Scalar normalize_angle(Scalar angle) {
                    while (angle > M_PI) {
                        angle -= 2 * M_PI;
                    }
                    while (angle < -M_PI) {
                        angle += 2 * M_PI;
                    }
                    return angle;
                }
            };

        }  // namespace filter
    }      // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_FILTER_COMPLEMENTARYFILTER_HPP
