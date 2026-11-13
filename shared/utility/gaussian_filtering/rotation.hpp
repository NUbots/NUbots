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

#ifndef UTILITY_GAUSSIAN_FILTERING_ROTATION_HPP
#define UTILITY_GAUSSIAN_FILTERING_ROTATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>  // tangent_basis uses cross products
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

namespace utility::gaussian_filtering {

    /// @brief Rotation matrix about the x axis by an angle in radians
    template <typename Scalar>
    Eigen::Matrix3<Scalar> rotx(const Scalar& x) {
        using std::cos, std::sin;
        Eigen::Matrix3<Scalar> R = Eigen::Matrix3<Scalar>::Identity();

        R(1, 1) = cos(x);
        R(1, 2) = -sin(x);
        R(2, 1) = sin(x);
        R(2, 2) = cos(x);
        return R;
    }

    /// @brief Rotation matrix about the y axis by an angle in radians
    template <typename Scalar>
    Eigen::Matrix3<Scalar> roty(const Scalar& x) {
        using std::cos, std::sin;
        Eigen::Matrix3<Scalar> R = Eigen::Matrix3<Scalar>::Identity();
        R(0, 0)                  = cos(x);
        R(0, 2)                  = sin(x);
        R(2, 0)                  = -sin(x);
        R(2, 2)                  = cos(x);
        return R;
    }

    /// @brief Rotation matrix about the z axis by an angle in radians
    template <typename Scalar>
    Eigen::Matrix3<Scalar> rotz(const Scalar& x) {
        using std::cos, std::sin;
        Eigen::Matrix3<Scalar> R = Eigen::Matrix3<Scalar>::Identity();
        R(0, 0)                  = cos(x);
        R(0, 1)                  = -sin(x);
        R(1, 0)                  = sin(x);
        R(1, 1)                  = cos(x);
        return R;
    }

    /// @brief Rotation matrix R = Rz * Ry * Rx from [roll, pitch, yaw] in radians
    template <typename Derived>
    Eigen::Matrix3<typename Derived::Scalar> rpy2rot(const Eigen::MatrixBase<Derived>& Theta) {
        using Scalar = typename Derived::Scalar;
        Eigen::Matrix3<Scalar> R;
        R = rotz(Theta(2)) * roty(Theta(1)) * rotx(Theta(0));
        return R;
    }

    /// @brief [roll, pitch, yaw] in radians from a rotation matrix
    template <typename Derived>
    Eigen::Vector3<typename Derived::Scalar> rot2rpy(const Eigen::MatrixBase<Derived>& R) {
        using Scalar = typename Derived::Scalar;
        using std::atan2, std::hypot;
        Eigen::Vector3<Scalar> Theta;
        Theta(0) = atan2(R(2, 1), R(2, 2));
        Theta(1) = atan2(-R(2, 0), hypot(R(2, 1), R(2, 2)));
        Theta(2) = atan2(R(1, 0), R(0, 0));
        return Theta;
    }

    /// @brief Rotation matrix from a quaternion (w, x, y, z), which is normalised first
    template <typename Derived>
    Eigen::Matrix3<typename Derived::Scalar> quat2rot(const Eigen::MatrixBase<Derived>& q) {
        using Scalar = typename Derived::Scalar;
        using std::sqrt;

        const Scalar n = sqrt(q(0) * q(0) + q(1) * q(1) + q(2) * q(2) + q(3) * q(3));
        const Scalar w = q(0) / n;
        const Scalar x = q(1) / n;
        const Scalar y = q(2) / n;
        const Scalar z = q(3) / n;

        Eigen::Matrix3<Scalar> R;
        R(0, 0) = Scalar(1) - Scalar(2) * (y * y + z * z);
        R(0, 1) = Scalar(2) * (x * y - z * w);
        R(0, 2) = Scalar(2) * (x * z + y * w);
        R(1, 0) = Scalar(2) * (x * y + z * w);
        R(1, 1) = Scalar(1) - Scalar(2) * (x * x + z * z);
        R(1, 2) = Scalar(2) * (y * z - x * w);
        R(2, 0) = Scalar(2) * (x * z - y * w);
        R(2, 1) = Scalar(2) * (y * z + x * w);
        R(2, 2) = Scalar(1) - Scalar(2) * (x * x + y * y);
        return R;
    }

    /// @brief Quaternion kinematics matrix Xi(q) (4x3): qdot = 0.5*quat_Xi(q)*omega_body
    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 4, 3> quat_Xi(const Eigen::MatrixBase<Derived>& q) {
        using Scalar    = typename Derived::Scalar;
        const Scalar& w = q(0);
        const Scalar& x = q(1);
        const Scalar& y = q(2);
        const Scalar& z = q(3);

        // clang-format off
        Eigen::Matrix<Scalar, 4, 3> Xi;
        Xi << -x, -y, -z,
               w, -z,  y,
               z,  w, -x,
              -y,  x,  w;
        // clang-format on
        return Xi;
    }

    /// @brief Unit quaternion (w, x, y, z) with w >= 0 from a rotation matrix
    inline Eigen::Vector4d rot2quat(const Eigen::Matrix3d& R) {
        const Eigen::Quaterniond q(R);
        // Eigen stores (x, y, z, w). q and -q are the same rotation, so use w >= 0.
        Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
        if (v(0) < 0.0) {
            v = -v;
        }
        return v.normalized();
    }

    /// @brief Unit quaternion (w, x, y, z) with w >= 0 from [roll, pitch, yaw] in radians
    inline Eigen::Vector4d rpy2quat(const Eigen::Vector3d& Theta) {
        return rot2quat(rpy2rot(Theta));
    }

    /// @brief An orthonormal basis (3x2) for the tangent plane of the unit sphere at u, in
    ///        which unit-ray residuals and their covariances are evaluated
    inline Eigen::Matrix<double, 3, 2> tangent_basis(const Eigen::Vector3d& u) {
        Eigen::Vector3d t1 = u.cross(Eigen::Vector3d::UnitZ());
        if (t1.squaredNorm() < 1e-8) {
            t1 = u.cross(Eigen::Vector3d::UnitX());
        }
        t1.normalize();
        Eigen::Matrix<double, 3, 2> T;
        T.col(0) = t1;
        T.col(1) = u.cross(t1).normalized();
        return T;
    }

}  // namespace utility::gaussian_filtering

#endif  // UTILITY_GAUSSIAN_FILTERING_ROTATION_HPP
