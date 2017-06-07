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

#ifndef UTILITY_MATH_COORDINATES_H
#define UTILITY_MATH_COORDINATES_H

#include <cmath>

namespace utility {
    namespace math {

        /**
         * Functions to convert between coordinate representations.
         *
         * (r,phi,theta) represent radial distance, bearing (counter-clockwise from x-axis in xy-plane) and elevation (measured from the xy plane) (in radians)
         * @author Alex Biddulph
         */
        namespace coordinates {
            inline Eigen::Vector3d sphericalToCartesian(const Eigen::Vector3d& sphericalCoordinates) {
                double distance = sphericalCoordinates[0];
                double cos_theta = cos(sphericalCoordinates[1]);
                double sin_theta = sin(sphericalCoordinates[1]);
                double cos_phi = cos(sphericalCoordinates[2]);
                double sin_phi = sin(sphericalCoordinates[2]);
                Eigen::Vector3d result;

                result[0] = distance * cos_theta * cos_phi;
                result[1] = distance * sin_theta * cos_phi;
                result[2] = distance * sin_phi;

                return result;
            }

            inline Eigen::Vector3d cartesianToSpherical(const Eigen::Vector3d& cartesianCoordinates)  {
                double x = cartesianCoordinates[0];
                double y = cartesianCoordinates[1];
                double z = cartesianCoordinates[2];
                Eigen::Vector3d result;

                result[0] = sqrt(x*x + y*y + z*z);  //r
                result[1] = atan2(y, x);            //theta
                if(result[0] == 0) {
                    result[2] = 0;
                } else {
                    result[2] = asin(z / (result[0]));  //phi
                }

                return result;
            }

            inline Eigen::Vector4d sphericalToCartesian4(const Eigen::Vector3d& sphericalCoordinates) {
                Eigen::Vector3d p = sphericalToCartesian(sphericalCoordinates);
                return Eigen::Vector4d(p[0],p[1],p[2],1);
            }

            inline Eigen::Vector4d cartesianToSpherical4(const Eigen::Vector3d& cartesianCoordinates)  {
                Eigen::Vector3d p = cartesianToSpherical(cartesianCoordinates);
                return Eigen::Vector4d(p[0],p[1],p[2],1);
            }


            inline Eigen::Vector2d cartesianToRadial(const Eigen::Vector2d& cartesianCoordinates)  {
                double x = cartesianCoordinates[0];
                double y = cartesianCoordinates[1];

                return { sqrt(x*x + y*y), atan2(y, x) };
            }

            inline Eigen::Vector2d spherical2Radial(const Eigen::Vector3d& sphericalCoordinates) {
                double dist = sphericalCoordinates(0);
                double declination = sphericalCoordinates(2);
                double flat_distance = dist * std::cos(declination);

                Eigen::Vector2d result;
                result[0] = flat_distance;
                result[1] = sphericalCoordinates(1);

                return result;
            }
        }
    }
}


#endif // UTILITY_MATH_COORDINATES_H

