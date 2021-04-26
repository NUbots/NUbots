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
     * All combinations
     * of Euler angles types
     * in same order as rotation application
     *
     * EulerYawPitchRoll is built as
     * Roll * Pitch * Yaw.
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
     * Valid Euler angles (0, 1, 2) range
     * are (bound included):
     * 0: -M_PI : M_PI
     * 1: -M_PI/2.0 : M_PI/2.0
     * 2: 0.0 : M_PI
     */

    /**
     * Return false if given Euler angles range are no valid
     */
    template <typename T, std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline bool CheckEulerBounds(const T& angles) {
        return (angles(0) >= -M_PI && angles(0) <= M_PI)
               && ((angles(1) > -M_PI / 2.0 && angles(1) < M_PI / 2.0)
                   || (angles(0) == 0 && angles(2) == 0 && (angles(1) == -M_PI / 2.0 || angles(1) == M_PI / 2.0)))
               && (angles(2) >= 0.0 && angles(2) <= M_PI);
    }

    /**
     * Convert given Euler angles of given
     * convention type to rotation matrix
     */
    template <typename T,
              typename Scalar                                                                 = typename T::Scalar,
              std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline Eigen::Matrix3d EulerToMatrix(const T& angles, EulerType eulerType) {
        Eigen::Quaternion<Scalar> quat;
        switch (eulerType) {
            case EulerYawPitchRoll: {
                Eigen::AngleAxis<Scalar> yawRot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitchRot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> rollRot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = rollRot * pitchRot * yawRot;
            } break;
            case EulerYawRollPitch: {
                Eigen::AngleAxis<Scalar> yawRot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitchRot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> rollRot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = pitchRot * rollRot * yawRot;
            } break;
            case EulerRollPitchYaw: {
                Eigen::AngleAxis<Scalar> yawRot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitchRot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> rollRot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = yawRot * pitchRot * rollRot;
            } break;
            case EulerRollYawPitch: {
                Eigen::AngleAxis<Scalar> yawRot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitchRot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> rollRot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = pitchRot * yawRot * rollRot;
            } break;
            case EulerPitchRollYaw: {
                Eigen::AngleAxis<Scalar> yawRot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitchRot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> rollRot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = yawRot * rollRot * pitchRot;
            } break;
            case EulerPitchYawRoll: {
                Eigen::AngleAxis<Scalar> yawRot(angles(1), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
                Eigen::AngleAxis<Scalar> pitchRot(angles(0), Eigen::Matrix<Scalar, 3, 1>::UnitY());
                Eigen::AngleAxis<Scalar> rollRot(angles(2), Eigen::Matrix<Scalar, 3, 1>::UnitX());
                quat = rollRot * yawRot * pitchRot;
            } break;
            default: {
                throw std::logic_error("Euler invalid type");
            }
        }
        return quat.matrix();
    }

    /**
     * Convert the given rotation matrix into
     * Euler angles of given convention
     */
    template <typename T,
              typename Scalar                                                                 = typename T::Scalar,
              std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 3))>* = nullptr>
    inline Eigen::Matrix<Scalar, 3, 1> MatrixToEuler(const T& mat, EulerType eulerType) {
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
     * Manually convert the given rotation matrix
     * to [Roll, Pitch, Yaw] ZYX intrinsic euler
     * angle (Better range than Eigen conversion).
     */
    template <typename T,
              typename Scalar                                                                 = typename T::Scalar,
              std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 3))>* = nullptr>
    inline Eigen::Matrix<Scalar, 3, 1> MatrixToEulerIntrinsic(const T& mat) {
        // Eigen euler angles and with better range)
        return Eigen::Matrix<Scalar, 3, 1>(
            // Roll
            std::atan2(mat(2, 1), mat(2, 2)),
            // Pitch
            std::atan2(-mat(2, 0), std::sqrt(mat(0, 0) * mat(0, 0) + mat(1, 0) * mat(1, 0))),
            // Yaw
            std::atan2(mat(1, 0), mat(0, 0)));
    }

    /**
     * Convert given Euler angles in [Roll, Pitch, Yaw]
     * ZYX intrinsic format to rotation matrix
     */
    template <typename T,
              typename Scalar                                                                 = typename T::Scalar,
              std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline Eigen::Matrix<Scalar, 3, 3> EulerIntrinsicToMatrix(const T& angles) {
        Eigen::AngleAxis<Scalar> yawRot(angles.z(), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
        Eigen::AngleAxis<Scalar> pitchRot(angles.y(), Eigen::Matrix<Scalar, 3, 1>::UnitY());
        Eigen::AngleAxis<Scalar> rollRot(angles.x(), Eigen::Matrix<Scalar, 3, 1>::UnitX());
        Eigen::Quaternion<Scalar> quat = yawRot * pitchRot * rollRot;
        return quat.matrix();
    }

}  // namespace utility::math::euler

#endif
