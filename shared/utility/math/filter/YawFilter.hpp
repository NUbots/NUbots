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

#ifndef UTILITY_MATH_FILTER_YAWFILTER_HPP
#define UTILITY_MATH_FILTER_YAWFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace utility {
    namespace math {
        namespace filter {

            template <typename Scalar>
            class YawFilter {
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

                /// @brief Whether the filter has been initialised
                bool initialised = false;

                /// @brief Previous kinematic yaw for bias estimation
                Scalar previous_kinematic_yaw = 0.0;

                /// @brief Maximum allowed bias
                Scalar max_bias = 0.1;

            public:
                /// @brief Constructor
                YawFilter(const Scalar alpha = 0.1, const Scalar beta = 0.01, const Scalar max_bias = 0.1)
                    : alpha(alpha), beta(beta), max_bias(max_bias) {}

                /// @brief Update the filter with gyroscope measurement and kinematic measurement
                /// @param gyro_z Z-axis gyroscope measurement (yaw rate)
                /// @param kinematic_yaw Yaw measurement from kinematics
                /// @param dt Time step
                /// @return Fused yaw estimate
                Scalar update(const Scalar gyro_z, const Scalar kinematic_yaw, const Scalar dt) {
                    if (!initialised) {
                        yaw                    = kinematic_yaw;
                        bias                   = 0.0;
                        previous_kinematic_yaw = kinematic_yaw;
                        initialised            = true;
                        return yaw;
                    }

                    // Calculate actual kinematic rate (change in kinematic yaw)
                    Scalar kinematic_rate = normalise_angle(kinematic_yaw - previous_kinematic_yaw) / dt;

                    // Update bias estimate: gyro should match kinematic rate when both are accurate
                    // The bias is the persistent error between gyro and actual rate
                    Scalar rate_error = (gyro_z - bias) - kinematic_rate;
                    bias += beta * rate_error;
                    bias = std::clamp(bias, -max_bias, max_bias);

                    // Remove bias from gyroscope measurement
                    Scalar corrected_gyro = gyro_z - bias;

                    // Predict yaw using bias-corrected gyroscope integration
                    Scalar predicted_gyro_yaw = yaw + corrected_gyro * dt;

                    // Predict kinematic yaw using difference
                    Scalar predicted_kinematic_yaw = yaw + (kinematic_yaw - previous_kinematic_yaw);

                    // Normalize angles to [-pi, pi]
                    Scalar normalized_predicted_gyro_yaw      = normalise_angle(predicted_gyro_yaw);
                    Scalar normalized_predicted_kinematic_yaw = normalise_angle(predicted_kinematic_yaw);

                    // Complementary filter update with proper angular interpolation
                    yaw = interpolate_angles(normalized_predicted_kinematic_yaw, normalized_predicted_gyro_yaw, alpha);
                    yaw = normalise_angle(yaw);

                    // Store current kinematic yaw for next iteration
                    previous_kinematic_yaw = kinematic_yaw;

                    return yaw;
                }

                /// @brief Update the filter with only gyroscope measurement
                /// @param gyro_z Z-axis gyroscope measurement (yaw rate)
                /// @param dt Time step
                /// @return Predicted yaw estimate
                Scalar update_gyro_only(const Scalar gyro_z, const Scalar dt) {
                    if (!initialised) {
                        return 0.0;
                    }

                    // Remove bias from gyroscope measurement
                    Scalar corrected_gyro = gyro_z - bias;

                    // Predict yaw using bias-corrected gyroscope integration only
                    yaw = yaw + corrected_gyro * dt;
                    yaw = normalise_angle(yaw);

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
                    yaw         = normalise_angle(new_yaw);
                    initialised = true;
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

                /// @brief Set the maximum allowed bias
                void set_max_bias(const Scalar new_max_bias) {
                    max_bias = new_max_bias;
                }

                /// @brief Get the complementary filter coefficient
                Scalar get_alpha() const {
                    return alpha;
                }

                /// @brief Get the bias learning rate
                Scalar get_beta() const {
                    return beta;
                }

                /// @brief Reset the filter
                void reset() {
                    yaw                    = 0.0;
                    bias                   = 0.0;
                    previous_kinematic_yaw = 0.0;
                    initialised            = false;
                }

            private:
                /// @brief Efficient angle normalization to [-pi, pi]
                static Scalar normalise_angle(Scalar angle) {
                    angle = std::fmod(angle + M_PI, 2 * M_PI);
                    return angle < 0 ? angle + M_PI : angle - M_PI;
                }

                /// @brief Interpolate between two angles handling wrapping correctly
                /// @param angle1 First angle
                /// @param angle2 Second angle
                /// @param weight Weight for angle1 (0-1)
                /// @return Interpolated angle
                static Scalar interpolate_angles(Scalar angle1, Scalar angle2, Scalar weight) {
                    // Calculate the angular difference, accounting for wrapping
                    Scalar diff = normalise_angle(angle1 - angle2);
                    // Interpolate the difference and add to angle2
                    return normalise_angle(angle2 + weight * diff);
                }
            };

        }  // namespace filter
    }      // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_FILTER_YAWFILTER_HPP
