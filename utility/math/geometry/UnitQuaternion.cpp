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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "UnitQuaternion.h"
#include "utility/math/angle.h"

namespace utility {
namespace math {
namespace geometry {

    UnitQuaternion::UnitQuaternion() {
        real() = 1;
        imaginary().zeros();
    }

    UnitQuaternion::UnitQuaternion(double realPart, const arma::vec3& imaginaryPart) {
        real()      = realPart;
        imaginary() = imaginaryPart;
    }

    UnitQuaternion::UnitQuaternion(double W, double X, double Y, double Z)
    {
        real()      = W;
        imaginary() = arma::vec3({X, Y, Z});
    }

    UnitQuaternion::UnitQuaternion(const arma::vec3& v) {
        real() = 0;
    	imaginary() = v;
    }

    UnitQuaternion::UnitQuaternion(const arma::vec3& vec1, const arma::vec3& vec2)
    {
        double norm     = arma::norm(vec1) * arma::norm(vec2);
        double half_cos = std::sqrt(0.5 + arma::dot(vec1, vec2) / (2.0 * norm));

        real()      = half_cos;
        imaginary() = arma::cross(vec1, vec2) / (2.0 * half_cos * norm);

        this->normalise();
    }

    UnitQuaternion::UnitQuaternion(const arma::vec3& axis, double angle) {
    	real() = std::cos(angle / 2.0);
    	imaginary() = std::sin(angle / 2.0) * arma::normalise(axis);
    }

    UnitQuaternion UnitQuaternion::i() const {
    	UnitQuaternion qi = *this;
        // take the congugate, as it is equal to the inverse when a unit vector
    	qi.imaginary() *= -1;
    	return qi;
    }

    arma::vec3 UnitQuaternion::rotateVector(const arma::vec3& v) const {
    	UnitQuaternion vRotated = *this * UnitQuaternion(v) * i();
        return vRotated.imaginary();
    }

    arma::vec3 UnitQuaternion::getAxis() const {
    	double angle = getAngle();
    	double sinThetaOnTwo = std::sin(angle / 2.0);
    	return imaginary() / sinThetaOnTwo;
    }

    double UnitQuaternion::getAngle() const {
        //Max and min prevent nand error, presumably due to computational limitations
    	return 2 * utility::math::angle::acos_clamped(std::fmin(1,std::fmax(real(),-1)));
    }

    void UnitQuaternion::setAngle(double angle) {
        real() = std::cos(angle / 2.0);
        imaginary() = std::sin(angle / 2.0) * arma::normalise(imaginary());
    }

    void UnitQuaternion::scaleAngle(double scale) {
        setAngle(getAngle() * scale);
    }

    void UnitQuaternion::normalise() {
        *this = arma::normalise(*this);
    }

    double UnitQuaternion::norm() {
        return kW() * kW() + kX() * kX() + kY() * kY() + kZ() * kZ();
    }

    arma::mat33 UnitQuaternion::toRotationMatrix()
    {
        // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix

        // Generate factors
        double xw = kX() * kW();
        double xx = kX() * kX();
        double xy = kX() * kY();
        double xz = kX() * kZ();

        double yw = kY() * kW();
        double yy = kY() * kY();
        double yz = kY() * kZ();

        double zw = kZ() * kW();
        double zz = kZ() * kZ();

        // Generate matrix
        arma::mat33 rot;
        rot << (1.0 - 2.0 * (yy + zz)) << (2.0 * (xy - zw))       << (2.0 * (xz + yw))       << arma::endr
            << (2.0 * (xy + zw))       << (1.0 - 2.0 * (xx + zz)) << (2.0 * (yz - xw))       << arma::endr
            << (2.0 * (xz - yw))       << (2.0 * (yz + xw))       << (1.0 - 2.0 * (xx + yy));
        return(rot);
    }

    UnitQuaternion UnitQuaternion::operator - (const UnitQuaternion& p) const {
        return *this * p.i();
    }

	UnitQuaternion UnitQuaternion::operator * (const UnitQuaternion& p) const {
		//From http://en.wikipedia.org/wiki/Quaternion#Quaternions_and_the_geometry_of_R3
        double realPart = real() * p.real() - arma::dot(imaginary(), p.imaginary());

        arma::vec3 imaginaryPart = arma::cross(imaginary(), p.imaginary())
                                 + p.real() *   imaginary()
                                 +   real() * p.imaginary();

        return UnitQuaternion(realPart, imaginaryPart);
	}

    UnitQuaternion UnitQuaternion::slerp(const UnitQuaternion& p, const double& t) {
        // See http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
        // Where qa = *this and qb = p

        double cosHalfTheta = kW() * p.kW() + kX() * p.kX() + kY() * p.kY() + kZ() * p.kZ();

        // If qa=qb or qa=-qb then theta = 0 and we can return qa
        if (std::abs(cosHalfTheta) >= 1.0) {
            return *this;
        }

        double halfTheta = utility::math::angle::acos_clamped(cosHalfTheta);
        double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);

        // If theta = 180 degrees then result is not fully defined
        // We could rotate around any axis normal to qa or qb
        if (std::abs(sinHalfTheta) < 0.001) {
            return *this * 0.5 + p * 0.5;
        }

        // Interpolate
        double ratioA = std::sin((1 - t) * halfTheta) / sinHalfTheta;
        double ratioB = std::sin(t * halfTheta) / sinHalfTheta;

        return *this * ratioA + p * ratioB;
    }

}
}
}
