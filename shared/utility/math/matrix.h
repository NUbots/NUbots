/*
 * This file is part of NUBots Utility.
 *
 * NUBots Utility is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots Utility is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots Utility.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_MATRIX_H
#define UTILITY_MATH_MATRIX_H

#include <armadillo>
#include <cmath>

namespace utility {
    namespace math {

        /**
         * Some general matrix utilities (generating rotation matrices).
         *
         * @author Alex Biddulph
         */
        namespace matrix {
            inline arma::mat33 xRotationMatrix(double angle) {
                arma::mat33 xRotMatrix;
                double cosA = cos(angle);
                double sinA = sin(angle);

                xRotMatrix << 0     << 0     << 0     << arma::endr
                           << 0     << cosA  << -sinA << arma::endr
                           << 0     << sinA  << cosA  << arma::endr;
                return xRotMatrix;
            }

            inline arma::mat33 yRotationMatrix(double angle) {
                arma::mat33 yRotMatrix;
                double cosA = cos(angle);
                double sinA = sin(angle);

                yRotMatrix << cosA  << 0     << sinA  << arma::endr
                           << 0     << 0     << 0     << arma::endr
                           << -sinA << 0     << cosA  << arma::endr;
                return yRotMatrix;
            }

            inline arma::mat33 zRotationMatrix(double angle) {
                arma::mat33 zRotMatrix;
                double cosA = cos(angle);
                double sinA = sin(angle);

                zRotMatrix << cosA  << -sinA << 0     << arma::endr
                           << sinA  << cosA  << 0     << arma::endr
                           << 0     << 0     << 0     << arma::endr;
                return zRotMatrix;
            }
        }
    }
}

#endif // UTILITY_MATH_COORDINATES_H
