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

#ifndef UTILITY_MATH_COORDINATES_H
#define UTILITY_MATH_COORDINATES_H

#include <armadillo>
#include <cmath>

namespace utility {
    namespace math {

        /**
         * Functions to convert between coordinate representations.
         *
         * @author Alex Biddulph
         */
        namespace coordinates {
            inline arma::vec3 Spherical2Cartesian(const arma::vec3& sphericalCoordinates) {
                double distance = sphericalCoordinates[0];
                double bearingcos = cos(sphericalCoordinates[1]);
                double bearingsin = sin(sphericalCoordinates[1]);
                double elevationcos = cos(sphericalCoordinates[2]);
                double elevationsin = sin(sphericalCoordinates[2]);
                arma::vec3 result;

                result[0] = distance * bearingcos * elevationcos;
                result[1] = distance * bearingsin * elevationcos;
                result[2] = distance * elevationsin;

                return result;
            }

            inline arma::vec3 Cartesian2Spherical(const arma::vec3& cartesianCoordinates)  {
                double x = cartesianCoordinates[0];
                double y = cartesianCoordinates[1];
                double z = cartesianCoordinates[2];
                arma::vec3 result;

                result[0] = sqrt(x*x + y*y + z*z);
                result[1] = atan2(y, x);
                result[2] = asin(z / (result[0]));

                return result;
            }
        }
    }
}

#endif // UTILITY_MATH_COORDINATES_H
