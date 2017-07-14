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

#ifndef UTILITY_MATH_GEOMETRY_CONE_H
#define UTILITY_MATH_GEOMETRY_CONE_H

#include <armadillo>

namespace utility {
namespace math {
    namespace geometry {

        template <int DIM = 3>
        class Cone {
        public:
            using Vector = arma::vec::fixed<DIM>;
            using Matrix = arma::mat::fixed<DIM, DIM>;

            // unit radius per unit distance
            double gradient;
            // Axis direction
            arma::vec::fixed<DIM> unit_axis;

            // X is the vectors in columns
            bool setFromPoints(const Matrix& X) {
                try {
                    unit_axis = arma::normalise(arma::solve(X.t(), arma::ones(DIM)));
                }
                catch (std::runtime_error e) {
                    return false;
                    std::cout
                        << __FILE__ << " : " << __LINE__
                        << " standard exception : cannot construct cone from points - they are redundant or coplanar."
                        << std::endl;
                }
                // Rise
                double proj_length = arma::dot(X.col(0), unit_axis);
                // Run
                double norm_length = std::sqrt(1 - proj_length * proj_length);
                // Gradient
                gradient = norm_length / proj_length;
                if (arma::dot(X.col(0), unit_axis) - arma::dot(X.col(1), unit_axis) > 0.00001
                    || arma::dot(X.col(1), unit_axis) - arma::dot(X.col(2), unit_axis) > 0.00001) {
                    std::cout << __FILE__ << " : " << __LINE__ << " CONE NOT FORMED." << std::endl;
                    return false;
                }
                return true;
            }

            Vector projectPoint(const Vector& p) const {
                // Get normalised p
                double norm_p = arma::norm(p);
                if (norm_p == 0) return arma::zeros(DIM);
                Vector p_unit = p / norm_p;

                // Get orth projection to cone axis to construct plane
                Vector orth_u_p = arma::normalise(p - unit_axis * arma::dot(p, unit_axis));

                // Construct a unit vector on the cone and in the plane of p
                Vector cone_vec = arma::normalise(unit_axis + gradient * orth_u_p);

                // Project point to cone vector
                return cone_vec * arma::dot(p, cone_vec);
            }

            float distanceToPoint(const Vector& p) const {
                Vector p_cone = projectPoint(p);
                Vector p_orth = p - p_cone;
                // Compute size of orth component
                return arma::norm(p_orth);
            }

            float dotDistanceToPoint(const Vector& p) const {
                Vector p_cone = projectPoint(p);
                // Compute size of orth component
                float norm_prod = arma::norm(p) * arma::norm(p_cone);
                if (norm_prod != 0) {
                    return std::fabs(arma::dot(p_cone, p) / norm_prod);
                }
                // Max distance from the cone
                else
                    return 1;
            }
        };
    }
}
}

#endif
