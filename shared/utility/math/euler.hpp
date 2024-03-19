/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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

/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef UTILITY_MATH_EULER_HPP
#define UTILITY_MATH_EULER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace utility::math::euler {

    /**
     * @brief All combinations of Euler angles types in same order as rotation application,
     * EulerYawPitchRoll is built as Roll * Pitch * Yaw.
     */
    enum EulerType {
        EulerYawPitchRoll,
        EulerYawRollPitch,
        EulerRollPitchYaw,
        EulerRollYawPitch,
        EulerPitchRollYaw,
        EulerPitchYawRoll,
    };

    /**
     * @brief Valid Euler angles (0, 1, 2) range are (bound included):
     * 0: -M_PI : M_PI
     * 1: -M_PI/2.0 : M_PI/2.0
     * 2: 0.0 : M_PI
     */

    /**
     * @brief Check if given Euler angles range are not valid
     * @param type Euler angles
     * @return false if angles are not valid
     */
    template <typename T, std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline bool check_euler_bounds(const T& angles) {
        return (angles(0) >= -M_PI && angles(0) <= M_PI)
               && ((angles(1) > -M_PI / 2.0 && angles(1) < M_PI / 2.0)
                   || (angles(0) == 0 && angles(2) == 0 && (angles(1) == -M_PI / 2.0 || angles(1) == M_PI / 2.0)))
               && (angles(2) >= 0.0 && angles(2) <= M_PI);
    }

    /**
     * @brief Convert given Euler angles of given convention type to rotation matrix
     * @param angles Euler angles
     * @param eulerType Euler angles convention type
     * @return Rotation matrix
     */
    template <typename T,
              typename Scalar                                                                 = typename T::Scalar,
              std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline Eigen::Matrix3d eul_to_mat(const T& angles, EulerType eulerType) {
        Eigen::Quaternion<Scalar> quat;
        switch (eulerType) {
            case EulerYawPitchRoll: {
                Eigen::AngleAxis<Scalar> yaw_rot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitch_rot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> roll_rot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = roll_rot * pitch_rot * yaw_rot;
            } break;
            case EulerYawRollPitch: {
                Eigen::AngleAxis<Scalar> yaw_rot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitch_rot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> roll_rot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = pitch_rot * roll_rot * yaw_rot;
            } break;
            case EulerRollPitchYaw: {
                Eigen::AngleAxis<Scalar> yaw_rot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitch_rot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> roll_rot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = yaw_rot * pitch_rot * roll_rot;
            } break;
            case EulerRollYawPitch: {
                Eigen::AngleAxis<Scalar> yaw_rot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitch_rot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> roll_rot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = pitch_rot * yaw_rot * roll_rot;
            } break;
            case EulerPitchRollYaw: {
                Eigen::AngleAxis<Scalar> yaw_rot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitch_rot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> roll_rot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = yaw_rot * roll_rot * pitch_rot;
            } break;
            case EulerPitchYawRoll: {
                Eigen::AngleAxis<Scalar> yaw_rot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitch_rot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> roll_rot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = roll_rot * yaw_rot * pitch_rot;
            } break;
            default: {
                throw std::logic_error("Euler invalid type");
            }
        }
        return quat.matrix();
    }

    /**
     * @brief Convert the given rotation matrix into Euler angles of given convention
     * @param mat Rotation matrix
     * @param eulerType Euler angles convention
     * @return Euler angles
     */
    template <typename T,
              typename Scalar                                                                 = typename T::Scalar,
              std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 3))>* = nullptr>
    inline Eigen::Matrix<Scalar, 3, 1> mat_to_eul(const T& mat, EulerType eulerType) {
        Eigen::Matrix<Scalar, 3, 1> tmp(0.0, 0.0, 0.0);
        switch (eulerType) {
            case EulerYawPitchRoll: {
                tmp = mat.eulerAngles(0, 1, 2);
            } break;
            case EulerYawRollPitch: {
                tmp = mat.eulerAngles(1, 0, 2);
            } break;
            case EulerRollPitchYaw: {
                tmp = mat.eulerAngles(2, 1, 0);
            } break;
            case EulerRollYawPitch: {
                tmp = mat.eulerAngles(1, 2, 0);
            } break;
            case EulerPitchRollYaw: {
                tmp = mat.eulerAngles(2, 0, 1);
            } break;
            case EulerPitchYawRoll: {
                tmp = mat.eulerAngles(0, 2, 1);
            } break;
        }
        return Eigen::Matrix<Scalar, 3, 1>(tmp(2), tmp(1), tmp(0));
    }

    /**
     * @brief Manually convert the given rotation matrix to [Roll, Pitch, Yaw] ZYX intrinsic euler angle (Better range
     * than Eigen conversion).
     * @param mat Rotation matrix
     * @return [Roll, Pitch, Yaw] ZYX intrinsic euler angle
     */
    template <typename T,
              typename Scalar                                                                 = typename T::Scalar,
              std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 3))>* = nullptr>
    inline Eigen::Matrix<Scalar, 3, 1> mat_to_eul_intrinsic(const T& mat) {
        // Eigen euler angles and with better range
        return Eigen::Matrix<Scalar, 3, 1>(
            // Roll
            std::atan2(mat(2, 1), mat(2, 2)),
            // Pitch
            std::atan2(-mat(2, 0), std::sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2))),
            // Yaw
            std::atan2(mat(1, 0), mat(0, 0)));
    }

    /**
     * @brief Convert given Euler angles in [Roll, Pitch, Yaw] ZYX intrinsic format to rotation matrix
     * @param angles [Roll, Pitch, Yaw] ZYX intrinsic euler angles
     * @return Rotation matrix corresponding to given euler angles
     */
    template <typename T,
              typename Scalar                                                                 = typename T::Scalar,
              std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline Eigen::Matrix<Scalar, 3, 3> eul_intrinsic_to_mat(const T& angles) {
        Eigen::AngleAxis<Scalar> yaw_rot(angles.z(), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
        Eigen::AngleAxis<Scalar> pitch_rot(angles.y(), Eigen::Matrix<Scalar, 3, 1>::UnitY());
        Eigen::AngleAxis<Scalar> roll_rot(angles.x(), Eigen::Matrix<Scalar, 3, 1>::UnitX());
        Eigen::Quaternion<Scalar> quat = yaw_rot * pitch_rot * roll_rot;
        return quat.matrix();
    }
}  // namespace utility::math::euler

#endif
