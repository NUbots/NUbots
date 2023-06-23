/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2020 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_FILTER_MAHONYFILTER_HPP
#define UTILITY_MATH_FILTER_MAHONYFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utility {
    namespace math {
        namespace filter {

            /**
             * @brief Compute skew of a vector
             * @param v: input vector
             * @return skew of v
             */
            template <typename Scalar>
            Eigen::Matrix<Scalar, 3, 3> skew(const Eigen::Matrix<Scalar, 3, 1>& v) {
                Eigen::Matrix<Scalar, 3, 3> skew;
                skew << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
                return skew;
            }

            /**
             * @brief The Mahony update uses the IMU data to compute the attitude. To see report on this, go to
             * "public/Projects/Final Year Projects/NUgus Pose Estimation" folder on the NAS.
             * @param acc:     accelerometer reading as a column vector, [a_x, a_y, a_z]'. g force
             * @param gyro:    gyroscope reading as  column vector, [w_x, w_y, w_z]'. angular velocities in rad/s.
             * @param ts:      Sample rate of this update.
             * @param Ki:      Integral gain
             * @param Kp:      Proportional gain
             * @param Hwb:     Homogeneous transformation matrix from world to body frame of IMU
             * @param bias:    gyroscope bias
             */
            template <typename Scalar>
            void MahonyUpdate(const Eigen::Matrix<Scalar, 3, 1>& acc,
                              const Eigen::Matrix<Scalar, 3, 1>& gyro,
                              Eigen::Isometry3d& Hwb,
                              const Scalar ts,
                              const Scalar Ki,
                              const Scalar Kp,
                              Eigen::Matrix<Scalar, 3, 1>& bias) {
                // Normalize the accelerometer reading
                Eigen::Matrix<Scalar, 3, 1> rGBb = acc.normalized();

                // Invert the rotation matrix to get the body-to-world transformation
                Eigen::Matrix<Scalar, 3, 3> Rbw = Hwb.rotation().transpose();

                // Rotate the world gravity vector in the world frame into the body frame
                Eigen::Matrix<Scalar, 3, 1> est_rGBb = Rbw * Eigen::Matrix<Scalar, 3, 1>::UnitZ();

                // Calculate the error between the measured and estimated acceleration vectors
                Eigen::Matrix<Scalar, 3, 3> a_corr = rGBb * est_rGBb.transpose() - est_rGBb * rGBb.transpose();
                Eigen::Matrix<Scalar, 3, 1> omega_mes =
                    -1 * Eigen::Matrix<Scalar, 3, 1>(a_corr(2, 1), a_corr(0, 2), a_corr(1, 0));

                // Integrate the error vector to estimate the gyro bias
                bias += Ki * omega_mes * ts;

                // Depolarizing the gyroscope bias
                Eigen::Matrix<Scalar, 3, 1> l_omega = gyro + Kp * omega_mes + bias;

                // Find the quaternions rate of change
                Eigen::Matrix<Scalar, 4, 4> ome = Eigen::Matrix<Scalar, 4, 4>::Zero();
                ome.block(0, 0, 3, 3)           = -skew(l_omega);
                ome.block(3, 0, 1, 3)           = -l_omega.transpose();
                ome.block(0, 3, 3, 1)           = l_omega;

                // Calculate integral to find the attitude quaternion
                Eigen::Matrix<Scalar, 4, 1> quat = Eigen::Quaternion<Scalar>(Hwb.linear()).coeffs();
                quat += ts * Eigen::Matrix<Scalar, 4, 1>(0.5 * ome * quat);

                // Set the rotation matrix
                Hwb.linear() = Eigen::Quaternion<Scalar>(quat(3), quat(0), quat(1), quat(2)).toRotationMatrix();
            }


        }  // namespace filter
    }      // namespace math
}  // namespace utility

#endif
