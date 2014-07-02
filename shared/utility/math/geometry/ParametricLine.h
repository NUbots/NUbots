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
#ifndef UTILITY_MATH_GEOMETRY_PARAMETRICLINE_H
#define UTILITY_MATH_GEOMETRY_PARAMETRICLINE_H

#include <armadillo>

namespace utility {
namespace math {
namespace geometry {

	template<int n>
	class ParametricLine {
	private:
		using Vector = arma::vec::fixed<n>;

	public:
		Vector direction;
		Vector point;
		ParametricLine(){}

		void setFromDirection(const Vector& direction_, const Vector& point_){
			if(arma::norm(direction_,1) <= 0){
				throw std::domain_error("ParametricLine::setFromDirection - Direction is zero vector!");
			}
			direction = arma::normalise(direction_);
			point = point_;
		}

		void setFromTwoPoints(const Vector& p1, const Vector& p2){
			if(arma::norm(p2-p1,1) <= 0){
				throw std::domain_error("ParametricLine::setFromTwoPoints - Two points are identical!");
			}
			direction = arma::normalise(p2 - p1);
			point = p1;
		} 

		double distanceToPoint(const Vector& p){
			Vector x = p - point;
			return arma::norm(x - arma::dot(x,direction) * direction);
		}

		double x(double y);
		double y(double x);
	};

	template <>
	double ParametricLine<2>::x(const double& y) {
		return (y - point[1]) * direction[0] / direction[1] + point[0];
	}
	template <>
	double ParametricLine<2>::y(const double& x) {
		return (x - point[0]) * direction[1] / direction[0] + point[1];
	}

}
}
}
#endif