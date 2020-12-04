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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_GEOMETRY_UNITQUATERNION_HPP
#define UTILITY_MATH_GEOMETRY_UNITQUATERNION_HPP

#include <armadillo>

#include "utility/math/matrix/Rotation3D.hpp"

namespace utility {
namespace math {
    namespace matrix {
        template <int Dimensions>
        class Rotation;
        using Rotation3D = Rotation<3>;
    }  // namespace matrix
    namespace geometry {

        class UnitQuaternion : public arma::vec4 {
            using arma::vec4::vec4;  // inherit constructors

        private:
            /* @brief Constructor for non-unit quaternion for purpose of point representation
             */
            UnitQuaternion(const arma::vec3& v);

        public:
            UnitQuaternion();

            UnitQuaternion(const matrix::Rotation3D& rotation);

            UnitQuaternion(double W, double X, double Y, double Z);

            UnitQuaternion(const arma::vec3& vec1, const arma::vec3& vec2);

            UnitQuaternion operator-(const UnitQuaternion& p) const;

            UnitQuaternion operator*(const UnitQuaternion& p) const;

            UnitQuaternion(double realPart, const arma::vec3& imaginaryPart);

            /*! @brief Creates quaternion which rotates about 3D axis by angle radians
             */
            UnitQuaternion(const arma::vec3& axis, double angle);

            /*! @brief Swaps quat to -quat if kW < 0
             */
            void rectify();

            /*! @brief Gets the inverse of the quaternion
             */
            UnitQuaternion i() const;

            arma::vec3 rotateVector(const arma::vec3& v) const;

            arma::vec3 getAxis() const;

            double getAngle() const;

            void setAngle(double angle);

            void scaleAngle(double scale);

            void normalise();

            /* @return Matrix Q(q) such that given another quaternion q', then
             * Q(q) * q' = q * q'
             */
            arma::mat44 getLeftQuatMultMatrix() const;

            /* @return Matrix W(q) such that given another quaternion q', then
             * W(q) * q' = q' * q
             */
            arma::mat44 getRightQuatMultMatrix() const;

            static float random(float a, float b);
            static UnitQuaternion getRandomU(float max_angle);
            static UnitQuaternion getRandomN(float stddev);

            double norm();

            // real part
            inline double kW() const {
                return at(0);
            };
            inline double& kW() {
                return at(0);
            };

            inline double kX() const {
                return at(1);
            };
            inline double& kX() {
                return at(1);
            };

            inline double kY() const {
                return at(2);
            };
            inline double& kY() {
                return at(2);
            };

            inline double kZ() const {
                return at(3);
            };
            inline double& kZ() {
                return at(3);
            };

            inline double real() const {
                return at(0);
            };
            inline double& real() {
                return at(0);
            };

            inline const arma::subview_col<double> imaginary() const {
                return rows(1, 3);
            }
            inline arma::subview_col<double> imaginary() {
                return rows(1, 3);
            }

            UnitQuaternion slerp(const UnitQuaternion& p, const double& t);
            static inline UnitQuaternion Identity() {
                return (arma::vec4({1.0, 0.0, 0.0, 0.0}));
            }
        };
    }  // namespace geometry
}  // namespace math
}  // namespace utility

#endif
