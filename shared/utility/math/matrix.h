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
         * @author Jake Fountain
         * @author Brendan Annable
         */
        namespace matrix {

            inline arma::mat33 xRotationMatrix(double angle) {
                double cosAngle = cos(angle);
                double sinAngle = sin(angle);
                arma::mat rot = {1           , 0           , 0           , 
                                 0           , cosAngle    , -sinAngle   , 
                                 0           , sinAngle    , cosAngle    };
                rot.reshape(3,3);
                return rot;
            }
            inline arma::mat33 yRotationMatrix(double angle) {
                double cosAngle = cos(angle);
                double sinAngle = sin(angle);
                arma::mat rot = {cosAngle    , 0           , sinAngle    , 
                                 0           , 1           , 0           , 
                                 -sinAngle   , 0           , cosAngle    };
                rot.reshape(3,3);
                return rot;
            }

            inline arma::mat33 zRotationMatrix(double angle) {
                double cosAngle = cos(angle);
                double sinAngle = sin(angle);
                arma::mat rot = {cosAngle    , -sinAngle   , 0           , 
                                 sinAngle    , cosAngle    , 0           , 
                                 0           , 0           , 1           };
                rot.reshape(3,3);
                return rot;
            }

            inline arma::mat xRotationMatrix(double angle, int size) {
                if (size <= 2) {
                    throw "Rotations in two dimensions cannot be done about the x-axis. Use the z-axis.";
                }
                arma::mat rot(size, size);
                rot.eye();
                rot.submat(0,2,0,2) = xRotationMatrix(angle);                     
                return rot;
            }

            inline arma::mat yRotationMatrix(double angle, int size) {
                if (size <= 2) {
                    throw "Rotations in two dimensions cannot be done about the y-axis. Use the z-axis.";
                }
                arma::mat rot(size, size);
                rot.eye();
                rot.submat(0,2,0,2) = yRotationMatrix(angle);                     
                return rot;
            }    

            inline arma::mat zRotationMatrix(double angle, int size) {
                if (size <= 2) {
                    return zRotationMatrix(angle).submat(0,1,0,1);                 
                }
                arma::mat rot(size, size);
                rot.eye();
                rot.submat(0,2,0,2) = zRotationMatrix(angle);                     
                return rot;
            }    
            
        }
    }
}

#endif // UTILITY_MATH_COORDINATES_H
