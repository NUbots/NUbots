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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_MATRIX_H
#define UTILITY_MATH_MATRIX_H

#include <Eigen/Geometry>
#include <armadillo>
#include <random>

/**
 * Matrix related helper methods
 *
 * @author Brendan Annable
 */

namespace utility {
namespace math {
    namespace matrix {


        /**
         * @brief Returns an orthogonal vec3 to a given vec3
         * See: http://a.pomf.se/egicug.pdf
         * Efficient Construction of Perpendicular Vectors without Branching
         * Michael M. Stark
         */
        inline arma::vec3 orthogonal(const arma::vec3& v) {
            const unsigned int uyx = std::signbit(std::abs(v[0]) - std::abs(v[1]));
            const unsigned int uzx = std::signbit(std::abs(v[0]) - std::abs(v[2]));
            const unsigned int uzy = std::signbit(std::abs(v[1]) - std::abs(v[2]));
            const unsigned int xm  = uyx & uzx;
            const unsigned int ym  = (1 ^ xm) & uzy;
            const unsigned int zm  = 1 ^ (xm & ym);
            return {zm * v[1] - ym * v[2], xm * v[2] - zm * v[0], ym * v[0] - xm * v[1]};
        }

        /**
         * @brief An alternative method for returning a orthogonal vec3 to a given vec3
         * See: http://lolengine.net/blog/2013/09/21/picking-orthogonal-vector-combing-coconuts
         */
        inline arma::vec3 orthogonal2(const arma::vec3& v) {
            return std::abs(v[0]) > std::abs(v[2]) ? arma::vec3({-v[1], v[0], 0}) : arma::vec3({0, -v[2], v[1]});
        }

        /**
         * @brief Returns an arbitary orthonormal vec3 to the given vec3
         */
        inline arma::vec3 orthonormal(const arma::vec3& v) {
            auto u = orthogonal(v);
            return arma::normalise(u);
        }


        inline Eigen::Quaterniond randomU(const double& max_angle) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> real_random(0.0, max_angle);
            std::uniform_real_distribution<> phi_random(0.0, 2.0 * M_PI);
            std::uniform_real_distribution<> theta_random(-1.0, 1.0);
            double angle = real_random(gen);

            // Get axis:
            double phi      = phi_random(gen);
            double costheta = theta_random(gen);
            double theta    = std::acos(costheta);
            double r        = 1;
            double x        = r * std::sin(theta) * std::cos(phi);
            double y        = r * std::sin(theta) * std::sin(phi);
            double z        = r * std::cos(theta);

            return Eigen::Quaterniond(angle, x, y, z);
        }

        inline Eigen::Quaterniond randomN(const double& stddev) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<> real_random(0.0, stddev);
            std::uniform_real_distribution<> phi_random(0.0, 2.0 * M_PI);
            std::uniform_real_distribution<> theta_random(-1.0, 1.0);
            double angle = real_random(gen);

            // Get axis:
            double phi      = phi_random(gen);
            double costheta = theta_random(gen);
            double theta    = std::acos(costheta);
            double r        = 1;
            double x        = r * std::sin(theta) * std::cos(phi);
            double y        = r * std::sin(theta) * std::sin(phi);
            double z        = r * std::cos(theta);

            return Eigen::Quaterniond(angle, x, y, z);
        }
    }  // namespace matrix
}  // namespace math
}  // namespace utility
#endif  // UTILITY_MATH_MATRIX_H
