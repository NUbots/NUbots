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
#ifndef UTILITY_MATH_FILTER_MAHONYFILTER_HPP
#define UTILITY_MATH_FILTER_MAHONYFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utility {
    namespace math {
        namespace filter {

            template <typename Scalar>
            class MahonyFilter {
            private:
                /// @brief Proportional gain
                Scalar Kp = 0;

                /// @brief Integral gain
                Scalar Ki = 0;

                /// @brief Gyroscope bias
                Eigen::Matrix<Scalar, 3, 1> bias = Eigen::Matrix<Scalar, 3, 1>::Zero();

                /// @brief Rotation matrix from world to body frame of IMU
                Eigen::Matrix<Scalar, 3, 3> Rwb = Eigen::Matrix<Scalar, 3, 3>::Identity();

                /// @brief Skew function
                static Eigen::Matrix<Scalar, 3, 3> skew(const Eigen::Matrix<Scalar, 3, 1>& v) {
                    Eigen::Matrix<Scalar, 3, 3> skew;
                    skew << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
                    return skew;
                }

            public:
                // Constructor
                MahonyFilter(const Scalar Kp                         = 0,
                             const Scalar Ki                         = 0,
                             const Eigen::Matrix<Scalar, 3, 1>& bias = Eigen::Matrix<Scalar, 3, 1>::Zero(),
                             const Eigen::Matrix<Scalar, 3, 3>& Rwb  = Eigen::Matrix<Scalar, 3, 3>::Identity())
                    : Kp(Kp), Ki(Ki), bias(bias), Rwb(Rwb) {}


                /// @brief Update function
                Eigen::Matrix<Scalar, 3, 3> update(const Eigen::Matrix<Scalar, 3, 1>& acc,
                                                   const Eigen::Matrix<Scalar, 3, 1>& gyro,
                                                   const Scalar dt) {
                    // Normalize the accelerometer reading
                    Eigen::Matrix<Scalar, 3, 1> uGBb = acc.normalized();

                    // Invert the rotation matrix to get the body-to-world transformation
                    Eigen::Matrix<Scalar, 3, 3> Rbw = Rwb.transpose();

                    // Rotate the world gravity vector in the world frame into the body frame
                    Eigen::Matrix<Scalar, 3, 1> est_uGBb = Rbw * Eigen::Matrix<Scalar, 3, 1>::UnitZ();

                    // Calculate the error between the measured and estimated acceleration vectors
                    Eigen::Matrix<Scalar, 3, 3> a_corr = uGBb * est_uGBb.transpose() - est_uGBb * uGBb.transpose();
                    Eigen::Matrix<Scalar, 3, 1> omega_mes =
                        -1 * Eigen::Matrix<Scalar, 3, 1>(a_corr(2, 1), a_corr(0, 2), a_corr(1, 0));

                    // Integrate the error vector to estimate the gyro bias
                    bias += Ki * omega_mes * dt;

                    // Depolarizing the gyroscope bias
                    Eigen::Matrix<Scalar, 3, 1> l_omega = gyro + Kp * omega_mes + bias;

                    // Find the quaternions rate of change
                    Eigen::Matrix<Scalar, 4, 4> ome = Eigen::Matrix<Scalar, 4, 4>::Zero();
                    ome.block(0, 0, 3, 3)           = -skew(l_omega);
                    ome.block(3, 0, 1, 3)           = -l_omega.transpose();
                    ome.block(0, 3, 3, 1)           = l_omega;
                    // Calculate integral to find the attitude quaternion
                    Eigen::Matrix<Scalar, 4, 1> quat = Eigen::Quaternion<Scalar>(Rwb).coeffs();
                    quat += dt * Eigen::Matrix<Scalar, 4, 1>(0.5 * ome * quat);

                    // Set the rotation matrix
                    Rwb = Eigen::Quaternion<Scalar>(quat.normalized()).toRotationMatrix();
                    return Rwb;
                }

                /// @brief Get the rotation matrix
                Eigen::Matrix<Scalar, 3, 3> get_state() const {
                    return Rwb;
                }

                /// @brief Set the rotation matrix
                void set_state(const Eigen::Matrix<Scalar, 3, 3>& Rwb) {
                    this->Rwb = Rwb;
                }
            };

        }  // namespace filter
    }      // namespace math
}  // namespace utility

#endif
