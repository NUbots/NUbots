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
#ifndef UTILITY_MATH_GEOMETRY_LINE_H
#define UTILITY_MATH_GEOMETRY_LINE_H

#include <armadillo>
#include "utility/math/geometry/ParametricLine.h"

namespace utility {
namespace math {
namespace geometry {

	template<int n>
	class Plane {
	private:
		using Vector = arma::vec::fixed<n>;

	public:
		Vector normal;
		Vector point;

		Plane(){}

		bool setFromNormal(Vector normal_, Vector point_){
			normal = arma::normalise(normal_);
			point = point_;
			return arma::norm(normal,1) > 0; 
		}

		bool setFrom3Points(Vector p1, Vector p2, Vector p3){
			point = p1;
			normal = arma::normalise(arma::cross(p2-p1,p3-p1));// Positive if p3 palmside (RHR) relative to p2
			return arma::norm(normal,1) > 0;
		}

		Vector intersect(ParametricLine<n> l){
			return arma::dot(point - l.point, normal) * l.direction / arma::dot(l.direction, normal) + l.point;   
		}
	};

}
}
}
#endif