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

#include <Eigen/Core>

namespace utility {
namespace math {
    namespace filter {

        // For quaternion q = w + ix + jy + kz, the vector is [x, y, z, w]
        // The Mahony update uses the IMU data to compute the attitude
        // INPUT
        // acc:     accelerometer reading as a column vector, [a_x, a_y, a_z]'. g force
        // gyro:    gyroscope reading as  column vector, [w_x, w_y, w_z]'. angular velocities in rad/s.
        // ts:      Sample rate of the sensor. 100Hz update rate would be ts = 0.01
        // Ki:      integral proportional gain
        // Kp:      integral proportional gain
        // quat:    quaternion representing the rotation of the torso. Hwt.
        // bias:
        void MahonyUpdate(Eigen::Vector3d acc,
                          Eigen::Vector3d gyro,
                          const double ts,
                          const double Ki,
                          const double Kp,
                          Eigen::Quaterniond& quat,
                          Eigen::Vector3d& bias) {
            // Normalise the acceleration vector
            acc.normalize();

            // Compute the 3 by 3 attitude matrix from the quaternion
            Eigen::Vector3d rho(quat.x(), quat.y(), quat.z());
            double q4           = quat.w();
            double rho_norm_squ = std::pow(rho.norm(), 2);
            // clang-format off
            Eigen::Matrix3d rho_x;
            rho_x << 0, -rho.z(),  rho.y(),
               rho.z(),       0 , -rho.x(),
              -rho.y(),  rho.x(),        0;
            // clang-format on

            Eigen::Matrix3d attitude = ((std::pow(q4, 2) - rho_norm_squ) * Eigen::Matrix3d::Identity())
                                       - (2 * q4 * rho_x) + (2 * rho * rho.transpose());

            // Calculate the sensor error
            // Reference vector for gravity
            Eigen::Vector3d r_acc(0, 0, 1);
            // Calculate estimated accelerometer reading
            Eigen::Vector3d est_acc = attitude * r_acc;
            // Calculate error between estimate and real
            Eigen::Matrix3d a_corr = acc * est_acc.transpose() - est_acc * acc.transpose();
            // Vex (inverse function of skew-symmetric function) the error
            Eigen::Vector3d omega_mes(a_corr(2, 1), a_corr(0, 2), a_corr(1, 0));
            omega_mes = -omega_mes;

            // Estimate the bias
            if (Ki > 0) {
                // Calculate bias
                bias += Ki * omega_mes * ts;
            }
            else {
                bias = Eigen::Vector3d::Zero();
            }

            // Calculate attitude using quaternion dynamics
            // Depolarizing the gyroscope bias
            Eigen::Vector3d l_omega = gyro + Kp * omega_mes + bias;

            // Find the quaternions rate of change
            // clang-format off
            Eigen::Matrix3d omega_x;
            omega_x <<         0, -l_omega.z(),  l_omega.y(),
                     l_omega.z(),            0, -l_omega.x(),
                    -l_omega.y(),  l_omega.x(),            0;
            // clang-format on
            Eigen::Matrix4d ome;
            ome.block<3, 3>(0, 0) = -omega_x;
            ome.block<1, 3>(3, 0) = -l_omega.transpose();
            ome.block<3, 1>(0, 3) = l_omega;
            ome(3, 3)             = 0;

            ome *= 0.5;
            Eigen::Vector4d quat_vec(quat.x(), quat.y(), quat.z(), quat.w());
            Eigen::Vector4d q_d(ome * quat_vec);

            // // Calculate integral to find the attitude quaternion
            quat_vec += ts * q_d;

            quat.x() = quat_vec(0);
            quat.y() = quat_vec(1);
            quat.z() = quat_vec(2);
            quat.w() = quat_vec(3);
            quat.normalize();
        }

    }  // namespace filter
}  // namespace math
}  // namespace utility
