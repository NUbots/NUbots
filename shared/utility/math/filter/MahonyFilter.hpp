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
            template <typename T>
            Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& v) {
                Eigen::Matrix<T, 3, 3> skew;
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
                              Eigen::Transform<Scalar, 3, Eigen::Isometry>& Hwb,
                              const Scalar ts,
                              const Scalar Ki,
                              const Scalar Kp,
                              Eigen::Matrix<Scalar, 3, 1>& bias) {
                // Normalize the accelerometer reading
                Eigen::Matrix<Scalar, 3, 1> rGTt = acc.normalized();

                // Invert the rotation matrix to get the body-to-world transformation
                Eigen::Matrix<Scalar, 3, 3> Rbw = Hwb.linear().transpose();

                // Rotate the world gravity vector in the world frame into the body frame
                Eigen::Matrix<Scalar, 3, 1> rGBb     = Eigen::Matrix<Scalar, 3, 1>::UnitZ();
                Eigen::Matrix<Scalar, 3, 1> est_rGTt = Rbw * rGBb;

                // Calculate the error between the measured and estimated acceleration vectors
                Eigen::Matrix<Scalar, 3, 3> a_corr = rGTt * est_rGTt.transpose() - est_rGTt * rGTt.transpose();
                Eigen::Matrix<Scalar, 3, 1> omega_mes =
                    -1 * Eigen::Matrix<Scalar, 3, 1>(a_corr(2, 1), a_corr(0, 2), a_corr(1, 0));


                // Integrate the error vector to estimate the gyro bias
                bias += Ki * omega_mes * ts;

                // Depolarizing the gyroscope bias
                Eigen::Matrix<Scalar, 3, 1> l_omega = gyro + Kp * omega_mes + bias;

                // Find the quaternions rate of change
                Eigen::Matrix<Scalar, 3, 3> omega_x = skew(l_omega);
                Eigen::Matrix<Scalar, 4, 4> ome     = Eigen::Matrix<Scalar, 4, 4>::Zero();
                ome.block(0, 0, 3, 3)               = -omega_x;
                ome.block(3, 0, 1, 3)               = -l_omega.transpose();
                ome.block(0, 3, 3, 1)               = l_omega;
                ome(3, 3)                           = 0;

                Eigen::Quaternion<Scalar> quat(Hwb.linear());
                Eigen::Matrix<Scalar, 4, 1> quat_vec(quat.x(), quat.y(), quat.z(), quat.w());
                Eigen::Matrix<Scalar, 4, 1> q_d(0.5 * ome * quat_vec);

                // Calculate integral to find the attitude quaternion
                quat_vec += ts * q_d;

                // Set quat (Rwt) and normalise
                quat = Eigen::Quaternion<Scalar>(quat_vec(3), quat_vec(0), quat_vec(1), quat_vec(2));
                quat.normalize();

                // Set the rotation matrix
                Hwb.linear() = quat.toRotationMatrix();
            }


        }  // namespace filter
    }      // namespace math
}  // namespace utility

#endif
