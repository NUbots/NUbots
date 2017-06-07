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

#ifndef UTILITY_MATH_GEOMETRY_UNITQUATERNION_H
#define UTILITY_MATH_GEOMETRY_UNITQUATERNION_H


#include "utility/math/matrix/Rotation3D.h"

namespace utility {
namespace math {
namespace matrix {
    template <int Dimensions>
    class Rotation;
    using Rotation3D = Rotation<3>;
}
namespace geometry {

    class UnitQuaternion : public Eigen::Vector4d {
        using Eigen::Vector4d::vec4; // inherit constructors

        private:

            /* @brief Constructor for non-unit quaternion for purpose of point representation
            */
            UnitQuaternion(const Eigen::Vector3d& v);

        public:
            UnitQuaternion();

            UnitQuaternion(const matrix::Rotation3D& rotation);

            UnitQuaternion(double W, double X, double Y, double Z);

            UnitQuaternion(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2);

            UnitQuaternion operator - (const UnitQuaternion& p) const;

            UnitQuaternion operator * (const UnitQuaternion& p) const;

            UnitQuaternion(double realPart, const Eigen::Vector3d& imaginaryPart);

            /*! @brief Creates quaternion which rotates about 3D axis by angle radians
            */
            UnitQuaternion(const Eigen::Vector3d& axis, double angle);

            /*! @brief Swaps quat to -quat if kW < 0
            */
            void rectify();

            /*! @brief Gets the inverse of the quaternion
            */
            UnitQuaternion i() const;

            Eigen::Vector3d rotateVector(const Eigen::Vector3d& v) const;

            Eigen::Vector3d getAxis() const;

            double getAngle() const;

            void setAngle(double angle);

            void scaleAngle(double scale);

            void normalise();

            /* @return Matrix Q(q) such that given another quaternion q', then
             * Q(q) * q' = q * q'
            */
            Eigen::Matrix4d getLeftQuatMultMatrix() const;

            /* @return Matrix W(q) such that given another quaternion q', then
             * W(q) * q' = q' * q
            */
            Eigen::Matrix4d getRightQuatMultMatrix() const;

            static float random(float a, float b);
            static UnitQuaternion getRandomU(float max_angle);
            static UnitQuaternion getRandomN(float stddev);

            double norm();

            // real part
            inline double kW() const { return at(0); };
            inline double& kW() { return at(0); };

            inline double kX() const { return at(1); };
            inline double& kX() { return at(1); };

            inline double kY() const { return at(2); };
            inline double& kY() { return at(2); };

            inline double kZ() const { return at(3); };
            inline double& kZ() { return at(3); };

            inline double real() const { return at(0); };
            inline double& real() { return at(0); };

            inline const arma::subview_col<double> imaginary() const { return rows(1,3); }
            inline arma::subview_col<double> imaginary() { return rows(1,3); }

            UnitQuaternion slerp(const UnitQuaternion& p, const double& t);
            static inline UnitQuaternion Identity() { return(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)); }
    };

}
}
}

#endif
