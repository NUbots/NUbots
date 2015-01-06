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


namespace utility {
namespace math {
namespace geometry {

    using matrix::Rotation3D;

    UnitQuaternion::UnitQuaternion() {
        zeros();
    }

    UnitQuaternion::UnitQuaternion(const arma::vec3& v) {
    	imaginary() = v;
    }

    UnitQuaternion::UnitQuaternion(const double& angle, const arma::vec3& axis) {
    	real() = std::cos(angle / 2.0);
    	imaginary() = std::sin(angle / 2.0) * arma::normalise(axis);
    }

    UnitQuaternion UnitQuaternion::i() {
    	UnitQuaternion qi = *this;
    	qi.imaginary() *= -1;
    	return qi;
    }

    arma::vec3 UnitQuaternion::rotateVector(const arma::vec3& v) {
    	UnitQuaternion vRotated = *this * UnitQuaternion(v) * i();
        return vRotated.imaginary();
    }

    arma::vec3 UnitQuaternion::getAxis() {
    	double angle = getAngle();
    	double sinThetaOnTwo = std::sin(angle / 2.0);
    	return imaginary() / sinThetaOnTwo;
    }

    double UnitQuaternion::getAngle() {
    	return 2 * std::acos(real());
    }

	UnitQuaternion UnitQuaternion::operator * (const UnitQuaternion& p) const {
		//From http://en.wikipedia.org/wiki/Quaternion#Quaternions_and_the_geometry_of_R3
		UnitQuaternion qDotP;
		qDotP[0] = arma::dot(imaginary(), p.imaginary());

		UnitQuaternion qsps;
		qsps[0] = real() * p.real();

		UnitQuaternion qspv;
		qspv.imaginary() = real() * p.imaginary();

		UnitQuaternion qvps;
		qvps.imaginary() = imaginary() * p.real();

		UnitQuaternion qCrossP;
		qCrossP.imaginary() = arma::cross(imaginary(), p.imaginary());

		return qsps - qDotP + qspv + qvps + qCrossP;
	}

    UnitQuaternion::operator Rotation3D() const {
        return Rotation3D(*this);
    }

}
}
}