/*
 * This file is part of the Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Rotation3D.h"
#include "matrix.h"
#include "utility/math/angle.h"
#include "utility/math/comparison.h"


namespace utility {
namespace math {
    namespace matrix {

        using geometry::UnitQuaternion;
        using utility::math::almost_equal;

        template <int N = 3>
        Rotation3D::Rotation(const Eigen::Matrix<double, N, N>& m) {  // : Eigen::Matrix3d(m.topLeftCorner<3, 3>())  {
            static_assert(N >= 3, "Can't make a 3D rotation matrix from anything less than a 3x3 matrix.");
            *this = m.template topLeftCorner<3, 3>();
            *this = this->orthogonalise();
        }

        Rotation3D::Rotation(const UnitQuaternion& q) {
            // quaternion to rotation conversion
            // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
            // http://en.wikipedia.org/wiki/Rotation_group_SO(3)#Quaternions_of_unit_norm
            *this << 1 - 2 * q.kY() * q.kY() - 2 * q.kZ() * q.kZ(), 2 * q.kX() * q.kY() - 2 * q.kZ() * q.kW(),
                2 * q.kX() * q.kZ() + 2 * q.kY() * q.kW(),  // row 1
                2 * q.kX() * q.kY() + 2 * q.kZ() * q.kW(), 1 - 2 * q.kX() * q.kX() - 2 * q.kZ() * q.kZ(),
                2 * q.kY() * q.kZ() - 2 * q.kX() * q.kW(),  // row 2
                2 * q.kX() * q.kZ() - 2 * q.kY() * q.kW(), 2 * q.kY() * q.kZ() + 2 * q.kX() * q.kW(),
                1 - 2 * q.kX() * q.kX() - 2 * q.kY() * q.kY();  // row 3
        }

        Rotation3D::Rotation(const Eigen::Vector3d& axis) {
            double normAxis = axis.norm();

            if (normAxis == 0) {
                // Axis has zero length
                std::cout << "utility::math::matrix::Rotation3D: WARNING Zero rotation axis given" << std::endl;
                setIdentity();
                return;
            }

            // Construct an othonormal basis
            col(0) = axis / normAxis;       // x axis
            col(1) = orthonormal(col(0));   // arbitary orthonormal vector
            col(2) = col(0).cross(col(1));  // third othogonal vector
        }

        Rotation3D::Rotation(const Eigen::Vector3d& axis, double angle) : Rotation(axis) {
            // Rotate by angle
            *this *= Rotation3D::createRotationX(angle) * i();
        }

        Rotation3D Rotation3D::orthogonalise() const {
            Rotation3D R;
            R.x() = this->x();
            R.z() = this->x().cross(this->y());
            R.y() = R.z().cross(R.x());
            return R;
        }

        Rotation3D Rotation3D::rotateX(double radians) const {
            return *this * createRotationX(radians);
        }

        Rotation3D Rotation3D::rotateY(double radians) const {
            return *this * createRotationY(radians);
        }

        Rotation3D Rotation3D::rotateZ(double radians) const {
            return *this * createRotationZ(radians);
        }

        Rotation3D Rotation3D::worldToLocal(const Rotation3D& reference) const {
            // http://en.wikipedia.org/wiki/Change_of_basis
            return reference.inverse() * (*this);
        }

        Rotation3D Rotation3D::localToWorld(const Rotation3D& reference) const {
            // http://en.wikipedia.org/wiki/Change_of_basis
            return reference * (*this);
        }

        Rotation3D Rotation3D::i() const {
            // http://en.wikipedia.org/wiki/Rotation_matrix#Multiplication
            // The inverse of a rotation matrix is its transpose, which is also a rotation matrix.
            return transpose();
        }

        AxisAngle Rotation3D::axisAngle() const {
            AxisAngle result;
            Eigen::EigenSolver<Eigen::Matrix3d> solver(*this);

            Axis axis;
            bool axisFound = false;
            for (ssize_t i = 0; i < solver.eigenvalues().size(); i++) {
                if (almost_equal(std::real(solver.eigenvalues()[i]), 1.0, 4)) {  // account for numeric imprecision
                    axis      = solver.eigenvectors().col(i).real();             // Axis of rotation
                    axisFound = true;
                }
            }

            if (axis.norm() == 0 || !axisFound) {
                throw std::domain_error("utility::math::matrix::Rotation3D::axisAngle: No rotation found");
            }

            // Construct an ONB
            Eigen::Vector3d s = orthonormal(axis);
            Eigen::Vector3d t = axis.cross(s);
            // Rotate s to calculate angle of rotation
            Eigen::Vector3d rs = *this * s;

            return {
                axis, std::atan2(rs.dot(t), rs.dot(s))  // Angle of rotation
            };
        }

        Eigen::Vector3d Rotation3D::eulerAngles() const {
            // See: http://staff.city.ac.uk/~sbbh653/publications/euler.pdf
            // Computing Euler angles from a rotation matrix
            // Gregory G. Slabaugh
            double roll, pitch, yaw;  // psi, theta, phi

            if (!almost_equal(std::abs(this->operator()(2, 0)), 1.0, 4)) {
                pitch           = -utility::math::angle::asin_clamped(this->operator()(2, 0));
                double cosPitch = std::cos(pitch);
                roll            = std::atan2(this->operator()(2, 1) / cosPitch, this->operator()(2, 2) / cosPitch);
                yaw             = std::atan2(this->operator()(1, 0) / cosPitch, this->operator()(0, 0) / cosPitch);
            }

            else {
                roll = std::atan2(this->operator()(0, 1), this->operator()(0, 2));
                yaw  = 0;

                if (almost_equal(this->operator()(2, 0), -1.0, 4)) {
                    pitch = M_PI_2;
                }

                else {
                    pitch = -M_PI_2;
                }
            }

            return {roll, pitch, yaw};
        }

        float Rotation3D::norm(Rotation3D T) {
            UnitQuaternion q = UnitQuaternion(T);
            // Get angle between -2Pi and 2pi
            float angle = q.getAngle();
            // Just want magnitude
            float theta = std::fabs(angle);
            // But rotating more that Pi in one direction is equivalent to a rotation in the other direction
            return std::fmin(2 * M_PI - theta, theta);
        }


        Rotation3D Rotation3D::createRotationX(double radians) {
            double c = std::cos(radians);
            double s = std::sin(radians);
            Rotation rotation;
            // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
            rotation << 1, 0, 0,  // row 1
                0, c, -s,         // row 2
                0, s, c;          // row 3
            return rotation;
        }

        Rotation3D Rotation3D::createRotationXJacobian(double radians) {
            double c = -std::sin(radians);
            double s = std::cos(radians);
            Rotation rotation;
            // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
            rotation << 1, 0, 0,  // row 1
                0, c, -s,         // row 2
                0, s, c;          // row 3
            return rotation;
        }

        Rotation3D Rotation3D::createRotationY(double radians) {
            double c = std::cos(radians);
            double s = std::sin(radians);
            Rotation rotation;
            // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
            rotation << c, 0, s,  // row 1
                0, 1, 0,          // row 2
                -s, 0, c;         // row 3
            return rotation;
        }

        Rotation3D Rotation3D::createRotationYJacobian(double radians) {
            double c = -std::sin(radians);
            double s = std::cos(radians);
            Rotation rotation;
            // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
            rotation << c, 0, s,  // row 1
                0, 1, 0,          // row 2
                -s, 0, c;         // row 3
            return rotation;
        }

        Rotation3D Rotation3D::createRotationZ(double radians) {
            double c = std::cos(radians);
            double s = std::sin(radians);
            Rotation rotation;
            // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
            rotation << c, -s, 0,  // row 1
                s, c, 0,           // row 2
                0, 0, 1;           // row 3
            return rotation;
        }

        Rotation3D Rotation3D::createRotationZJacobian(double radians) {
            double c = -std::sin(radians);
            double s = std::cos(radians);
            Rotation rotation;
            // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
            rotation << c, -s, 0,  // row 1
                s, c, 0,           // row 2
                0, 0, 1;           // row 3
            return rotation;
        }


        Rotation3D Rotation3D::createFromEulerAngles(const Eigen::Vector3d& a) {
            // double roll = a[0];
            // double pitch = a[1];
            // double yaw = a[2];
            return Rotation3D::createRotationZ(a[2]) * Rotation3D::createRotationY(a[1])
                   * Rotation3D::createRotationX(a[0]);
        }
    }
}
}
