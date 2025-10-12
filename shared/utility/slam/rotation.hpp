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

#ifndef UTILITY_SLAM_ROTATION_HPP
#define UTILITY_SLAM_ROTATION_HPP

#include <Eigen/Core>

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

namespace utility::slam {

    /**
     * @brief Computes a rotation matrix around the X-axis.
     *
     * @tparam Scalar The scalar type for the rotation angle.
     * @param x The rotation angle in radians.
     * @return A 3x3 rotation matrix.
     */
    template <typename Scalar>
    Eigen::Matrix3<Scalar> rotx(const Scalar& x) {
        using std::cos, std::sin;
        Eigen::Matrix3<Scalar> R = Eigen::Matrix3<Scalar>::Identity();

        // Rotation around X-axis:
        // R = [1   0      0    ]
        //     [0  cos(x) -sin(x)]
        //     [0  sin(x)  cos(x)]
        R(1, 1) = cos(x);
        R(1, 2) = -sin(x);
        R(2, 1) = sin(x);
        R(2, 2) = cos(x);
        return R;
    }

    /**
     * @brief Computes a rotation matrix around the X-axis with its derivative.
     *
     * @tparam Scalar The scalar type for the rotation angle.
     * @param x The rotation angle in radians.
     * @param dRdx Output parameter for the derivative of the rotation matrix with respect to x.
     * @return A 3x3 rotation matrix.
     */
    template <typename Scalar>
    Eigen::Matrix3<Scalar> rotx(const Scalar& x, Eigen::Matrix3<Scalar>& dRdx) {
        using std::cos, std::sin;
        dRdx = Eigen::Matrix3<Scalar>::Zero();

        dRdx(1, 1) = -sin(x);
        dRdx(2, 1) = cos(x);

        dRdx(1, 2) = -cos(x);
        dRdx(2, 2) = -sin(x);
        return rotx(x);
    }

    /**
     * @brief Computes a rotation matrix around the Y-axis.
     *
     * @tparam Scalar The scalar type for the rotation angle.
     * @param x The rotation angle in radians.
     * @return A 3x3 rotation matrix.
     */
    template <typename Scalar>
    Eigen::Matrix3<Scalar> roty(const Scalar& x) {
        using std::cos, std::sin;
        Eigen::Matrix3<Scalar> R = Eigen::Matrix3<Scalar>::Identity();
        // Rotation around Y-axis:
        // R = [ cos(y)  0  sin(y)]
        //     [ 0      1  0    ]
        //     [-sin(y) 0  cos(y)]
        R(0, 0) = cos(x);
        R(0, 2) = sin(x);
        R(2, 0) = -sin(x);
        R(2, 2) = cos(x);
        return R;
    }

    /**
     * @brief Computes a rotation matrix around the Y-axis with its derivative.
     *
     * @tparam Scalar The scalar type for the rotation angle.
     * @param x The rotation angle in radians.
     * @param dRdx Output parameter for the derivative of the rotation matrix with respect to x.
     * @return A 3x3 rotation matrix.
     */
    template <typename Scalar>
    Eigen::Matrix3<Scalar> roty(const Scalar& x, Eigen::Matrix3<Scalar>& dRdx) {
        using std::cos, std::sin;
        dRdx = Eigen::Matrix3<Scalar>::Zero();

        dRdx(0, 0) = -sin(x);
        dRdx(2, 0) = -cos(x);

        dRdx(0, 2) = cos(x);
        dRdx(2, 2) = -sin(x);
        return roty(x);
    }

    /**
     * @brief Computes a rotation matrix around the Z-axis.
     *
     * @tparam Scalar The scalar type for the rotation angle.
     * @param x The rotation angle in radians.
     * @return A 3x3 rotation matrix.
     */
    template <typename Scalar>
    Eigen::Matrix3<Scalar> rotz(const Scalar& x) {
        using std::cos, std::sin;
        Eigen::Matrix3<Scalar> R = Eigen::Matrix3<Scalar>::Identity();
        // Rotation around Z-axis:
        // R = [ cos(z) -sin(z) 0]
        //     [ sin(z)  cos(z) 0]
        //     [ 0       0      1]
        R(0, 0) = cos(x);
        R(0, 1) = -sin(x);
        R(1, 0) = sin(x);
        R(1, 1) = cos(x);
        return R;
    }

    /**
     * @brief Computes a rotation matrix around the Z-axis with its derivative.
     *
     * @tparam Scalar The scalar type for the rotation angle.
     * @param x The rotation angle in radians.
     * @param dRdx Output parameter for the derivative of the rotation matrix with respect to x.
     * @return A 3x3 rotation matrix.
     */
    template <typename Scalar>
    Eigen::Matrix3<Scalar> rotz(const Scalar& x, Eigen::Matrix3<Scalar>& dRdx) {
        using std::cos, std::sin;
        dRdx = Eigen::Matrix3<Scalar>::Zero();

        dRdx(0, 0) = -sin(x);
        dRdx(1, 0) = cos(x);

        dRdx(0, 1) = -cos(x);
        dRdx(1, 1) = -sin(x);
        return rotz(x);
    }

    /**
     * @brief Converts roll-pitch-yaw angles to a rotation matrix.
     *
     * The rotation matrix is computed as R = Rz * Ry * Rx.
     *
     * @tparam Derived The derived type of the input Eigen expression.
     * @param Theta Vector containing [roll, pitch, yaw] angles in radians.
     * @return A 3x3 rotation matrix.
     */
    template <typename Derived>
    Eigen::Matrix3<typename Derived::Scalar> rpy2rot(const Eigen::MatrixBase<Derived>& Theta) {
        using Scalar = typename Derived::Scalar;
        // R = Rz*Ry*Rx
        Eigen::Matrix3<Scalar> R;
        R = rotz(Theta(2)) * roty(Theta(1)) * rotx(Theta(0));
        return R;
    }

    /**
     * @brief Converts a rotation matrix to roll-pitch-yaw angles.
     *
     * @tparam Derived The derived type of the input Eigen expression.
     * @param R A 3x3 rotation matrix.
     * @return Vector containing [roll, pitch, yaw] angles in radians.
     */
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

    /**
     * @brief Computes the kinematic transformation matrix T(theta).
     *
     * The transformation relates body-frame angular velocities to Euler angle rates.
     *
     * @tparam Scalar The scalar type for computations.
     * @param Thetanb Vector containing [roll, pitch, yaw] angles in radians.
     * @return A 3x3 kinematic transformation matrix.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> TK(const Eigen::Matrix<Scalar, 3, 1>& Thetanb) {
        Scalar phi   = Thetanb(0);
        Scalar theta = Thetanb(1);
        Eigen::Matrix<Scalar, 3, 3> TK = Eigen::Matrix<Scalar, 3, 3>::Zero();
        using std::cos, std::sin, std::tan;
        Scalar cphi   = cos(phi);
        Scalar sphi   = sin(phi);
        Scalar ctheta = cos(theta);
        Scalar ttheta = tan(theta);
        TK(0, 0)      = 1;
        TK(0, 1)      = sphi * ttheta;
        TK(0, 2)      = cphi * ttheta;
        TK(1, 1)      = cphi;
        TK(1, 2)      = -sphi;
        TK(2, 1)      = sphi / ctheta;
        TK(2, 2)      = cphi / ctheta;
        return TK;
    }

    /**
     * @brief Builds the complete 6x6 Euler kinematic transformation matrix.
     *
     * The matrix relates body-frame velocities to the time derivative of the pose vector.
     * J(eta) = [R_nb(theta_nb)    0]
     *          [0                 T(theta_nb)]
     *
     * @tparam Scalar The scalar type for computations.
     * @param eta The 6D pose vector [position; orientation] where orientation is [roll, pitch, yaw].
     * @return A 6x6 kinematic transformation matrix.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> eulerKinematicTransformation(const Eigen::Matrix<Scalar, 6, 1>& eta) {
        Eigen::Matrix<Scalar, 3, 1> thetanb = eta.template segment<3>(3);
        Eigen::Matrix<Scalar, 3, 3> Rnb     = rpy2rot(thetanb);
        Eigen::Matrix<Scalar, 3, 3> T       = TK(thetanb);

        Eigen::Matrix<Scalar, 6, 6> J = Eigen::Matrix<Scalar, 6, 6>::Zero();
        J.template block<3, 3>(0, 0)  = Rnb;
        J.template block<3, 3>(3, 3)  = T;
        return J;
    }

}  // namespace utility::slam

#endif  // UTILITY_SLAM_ROTATION_HPP
