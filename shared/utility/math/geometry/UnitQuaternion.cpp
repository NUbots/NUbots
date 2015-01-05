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

    constexpr uint kW = 0;   //real part
    constexpr uint kX = 1;
    constexpr uint kY = 2;
    constexpr uint kZ = 3;

    UnitQuaternion::UnitQuaternion(const arma::vec4& q_){
    	q = q_;
    }

    UnitQuaternion::UnitQuaternion(const arma::vec3& v){
    	q.rows(kX,kZ) = v;
    }

    UnitQuaternion::UnitQuaternion(const double& angle, const arma::vec3& axis){
    	q[kW] = std::cos(angle / 2.0);
    	q.rows(kX,kZ) = std::sin(angle / 2.0) * arma::normalise(axis);
    }

    UnitQuaternion UnitQuaternion::i(){
    	arma::vec4 qi = q;
    	qi.rows(kX,kZ) *= -1;
    	return UnitQuaternion(qi);
    }

    arma::vec3 UnitQuaternion::rotateVector(const arma::vec3& v){
    	UnitQuaternion vRotated = UnitQuaternion(q) * UnitQuaternion(v) * i();
        return vRotated.q.rows(kX,kZ);
    }

    arma::vec3 UnitQuaternion::getAxis(){
    	double angle = getAngle();
    	double sinThetaOnTwo = std::sin(angle / 2.0);
    	return q.rows(kX,kZ) / sinThetaOnTwo;
    }

    double UnitQuaternion::getAngle(){
    	return 2 * std::acos(q[kW]);
    }

    arma::vec UnitQuaternion::rows(const uint& i, const uint& j) const{
        return q.rows(i,j);
    }

    double UnitQuaternion::operator [] (const uint& i) const{
        return q[i];
    }


	UnitQuaternion UnitQuaternion::operator * (const UnitQuaternion& p) const{
		//From http://en.wikipedia.org/wiki/Quaternion#Quaternions_and_the_geometry_of_R3
		arma::vec4 qDotP;
		qDotP[0] = arma::dot(q.rows(kX,kZ), p.rows(kX,kZ));
		arma::vec4 qsps;
		qsps[0] = q[kW] * p[kW];
		arma::vec4 qspv;
		qspv.rows(kX,kZ) = q[kW] * p.rows(kX,kZ);
		arma::vec4 qvps;
		qvps.rows(kX,kZ) = q.rows(kX,kZ) * p[kW];
		arma::vec4 qCrossP;
		qCrossP.rows(kX,kZ) = arma::cross(q.rows(kX,kZ), p.rows(kX,kZ));
		return UnitQuaternion(arma::vec4(qsps - qDotP + qspv + qvps + qCrossP));
	}

    UnitQuaternion::operator Rotation3D() const {
        return Rotation3D(q);
    }

}
}
}