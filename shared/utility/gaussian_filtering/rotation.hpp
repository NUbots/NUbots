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
#include <Eigen/Geometry>  // tangentBasis uses cross products
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

namespace utility::gaussian_filtering {

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
     * @brief Rotation matrix from a quaternion (w, x, y, z).
     *
     * The quaternion is normalised inside, which is what makes every geometric model
     * built on this function invariant to |q|. That invariance is deliberate: it
     * confines the redundant fourth degree of freedom to a direction no bearing,
     * gravity or height measurement can see, so it cannot corrupt the attitude
     * estimate. MeasurementQuaternionNorm is what supplies information along it, and
     * without that the MAP Hessian would be singular there.
     *
     * @tparam Derived The derived type of the input Eigen expression.
     * @param q Quaternion (w, x, y, z); need not be unit length.
     * @return The 3x3 rotation matrix Rfb.
     */
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

    /**
     * @brief Quaternion kinematics matrix: qdot = 0.5*quatXi(q)*omega_body.
     *
     * Follows from qdot = 0.5*q (x) (0, omega_b), the body-rate form matching
     * Rdot = R*hatSO3(omega_b) for R = quat2rot(q). Unlike the roll-pitch-yaw rate
     * transform TK() it has no singularity: every entry is linear in q, so a robot
     * toppling through pitch = +-90 deg is an ordinary point on the trajectory.
     *
     * @tparam Derived The derived type of the input Eigen expression.
     * @param q Quaternion (w, x, y, z).
     * @return The 4x3 kinematics matrix Xi(q).
     */
    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 4, 3> quatXi(const Eigen::MatrixBase<Derived>& q) {
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

    /**
     * @brief Hamilton product of two (w, x, y, z) quaternions.
     *
     * @param a Left quaternion (w, x, y, z).
     * @param b Right quaternion (w, x, y, z).
     * @return The product a (x) b.
     */
    template <typename DerivedA, typename DerivedB>
    Eigen::Vector4<typename DerivedA::Scalar> quatMultiply(const Eigen::MatrixBase<DerivedA>& a,
                                                           const Eigen::MatrixBase<DerivedB>& b) {
        using Scalar = typename DerivedA::Scalar;
        Eigen::Vector4<Scalar> c;
        c(0) = a(0) * b(0) - a(1) * b(1) - a(2) * b(2) - a(3) * b(3);
        c(1) = a(0) * b(1) + a(1) * b(0) + a(2) * b(3) - a(3) * b(2);
        c(2) = a(0) * b(2) - a(1) * b(3) + a(2) * b(0) + a(3) * b(1);
        c(3) = a(0) * b(3) + a(1) * b(2) - a(2) * b(1) + a(3) * b(0);
        return c;
    }

    /**
     * @brief Quaternion (w, x, y, z) from a rotation matrix.
     *
     * @param R A 3x3 rotation matrix.
     * @return The unit quaternion (w, x, y, z) with w >= 0.
     */
    inline Eigen::Vector4d rot2quat(const Eigen::Matrix3d& R) {
        const Eigen::Quaterniond q(R);
        // Eigen stores (x, y, z, w); the sign is free, so pick w >= 0 for a canonical
        // representative -- q and -q are the same rotation, and letting the mean drift
        // between the two hemispheres would make a Gaussian over the components meaningless.
        Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
        if (v(0) < 0.0) {
            v = -v;
        }
        return v.normalized();
    }

    /**
     * @brief Quaternion (w, x, y, z) from roll-pitch-yaw angles.
     *
     * @param Theta Vector containing [roll, pitch, yaw] angles in radians.
     * @return The unit quaternion (w, x, y, z) with w >= 0.
     */
    inline Eigen::Vector4d rpy2quat(const Eigen::Vector3d& Theta) {
        return rot2quat(rpy2rot(Theta));
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
        Scalar phi                     = Thetanb(0);
        Scalar theta                   = Thetanb(1);
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

    /**
     * @brief An orthonormal basis for the tangent plane of the unit sphere at u.
     *
     * This is the 2D space in which unit-ray residuals and their covariances live: a
     * unit ray has only two degrees of freedom, so association surprisals are
     * evaluated there rather than in the rank-deficient 3D chordal space.
     *
     * @param u Unit vector
     * @return 3x2 matrix whose columns are orthonormal and perpendicular to u
     */
    inline Eigen::Matrix<double, 3, 2> tangentBasis(const Eigen::Vector3d& u) {
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

#endif  // UTILITY_SLAM_ROTATION_HPP
