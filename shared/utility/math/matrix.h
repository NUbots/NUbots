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

#ifndef UTILITY_MATH_MATRIX_H
#define UTILITY_MATH_MATRIX_H

#include <nuclear>
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

            inline arma::mat22 rotationMatrix(double angle) {
                double cosAngle = std::cos(angle);
                double sinAngle = std::sin(angle);
                arma::mat22 result;
                result << cosAngle    << -sinAngle   <<  arma::endr
                       << sinAngle    << cosAngle;
                return result;
            }

            inline arma::mat33 xRotationMatrix(double angle) {
                double cosAngle = cos(angle);
                double sinAngle = sin(angle);
                arma::mat33 result;
                result << 1           << 0           << 0           <<  arma::endr
                       << 0           << cosAngle    << -sinAngle   <<  arma::endr
                       << 0           << sinAngle    << cosAngle;
                return result;
            }
            inline arma::mat33 yRotationMatrix(double angle) {
                double cosAngle = cos(angle);
                double sinAngle = sin(angle);
                arma::mat33 result;
                result << cosAngle    << 0           << sinAngle    <<  arma::endr
                       << 0           << 1           << 0           << arma::endr
                       << -sinAngle   << 0           << cosAngle;
                return result;
            }

            inline arma::mat33 zRotationMatrix(double angle) {
                double cosAngle = cos(angle);
                double sinAngle = sin(angle);
                arma::mat33 result;
                result << cosAngle    << -sinAngle   << 0           <<  arma::endr
                       << sinAngle    << cosAngle    << 0           <<  arma::endr
                       << 0           << 0           << 1;
                return result;
            }

            inline arma::mat xRotationMatrix(double angle, int size) {
                if (size <= 2) {
                    throw std::runtime_error("Rotations in two dimensions cannot be done about the x-axis. Use the z-axis.");
                }
                arma::mat rot(size, size, arma::fill::eye);
                rot.submat(0,0,2,2) = xRotationMatrix(angle);
                return rot;
            }

            inline arma::mat yRotationMatrix(double angle, int size) {
                if (size <= 2) {
                    throw std::runtime_error("Rotations in two dimensions cannot be done about the y-axis. Use the z-axis.");
                }
                arma::mat rot(size, size, arma::fill::eye);
                rot.submat(0,0,2,2) = yRotationMatrix(angle);
                return rot;
            }

            inline arma::mat zRotationMatrix(double angle, int size) {
                if (size <= 2) {
                    return zRotationMatrix(angle).submat(0,1,0,1);
                }
                arma::mat rot(size, size, arma::fill::eye);
                rot.submat(0,0,2,2) = zRotationMatrix(angle);
                return rot;
            }

            inline arma::mat44 translationMatrix(arma::vec3 v){
                arma::mat44 result = arma::eye(4,4);
                result.col(3).rows(0,2) = v;
                return result;
            }

            inline arma::mat33 axisAngleRotationMatrix(arma::vec3 axis, double angle){
                //Construct appropriate ONB:
                arma::mat33 B;
                //Check axis not zero
                double normAxis = arma::norm(axis,2);
                if(normAxis != 0){
                    B.col(0) = axis/normAxis;
                } else {
                    NUClear::log<NUClear::WARN>("utility::math::matrix::axisAngleRotationMatrix - WARNING Zero rotation axis given");
                    return arma::eye(3,3);
                }

                //Get first orthogonal vector
                B.col(1) = arma::vec3({0, B.col(0)[2], -B.col(0)[1]});  //orthogonal to col0, unless zero
                double col1Norm = arma::norm(B.col(1), 2);
                if(col1Norm == 0){
                    B.col(1) = arma::vec3({B.col(0)[1], -B.col(0)[0],0});   //orthogonal to col0
                    B.col(1) *= (1/arma::norm(B.col(1),2));
                } else {
                    B.col(1) *= (1/col1Norm);
                }

                //Get second orthogonal vector
                B.col(2) = arma::cross(B.col(0),B.col(1));

                return B * xRotationMatrix(angle) * B.t();
            }
            /*! @return Pair containing the axis of the rotation as a unit vector followed by the rotation angle.*/
            inline std::pair<arma::vec3, double> axisAngleFromRotationMatrix(arma::mat33 matrix){
                std::pair<arma::vec3, double> result;
                arma::cx_vec eigValues;
                arma::cx_mat eigVectors;
                eig_gen(eigValues,eigVectors, matrix);

                for(size_t i = 0; i < eigValues.size(); i++){
                    if(std::real(eigValues[i])==1){
                        result.first = arma::real(eigVectors.col(i));   //Set axis of rotation for return
                    }
                }
                double norm = arma::norm(result.first,2);
                if(norm!=0){
                    result.first *= 1/norm;
                } else {
                    NUClear::log<NUClear::ERROR>("utility::math::matrix::axisAngleRotationMatrix -  ERROR :  No rotation found");
                    return result;
                }

                //Construct an ONB
                arma::vec3 s = {0,-result.first[2],result.first[1]};    //orth to result.first
                double sNorm = arma::norm(s,2);
                if(sNorm == 0){
                    s = arma::vec({result.first[1],-result.first[0],0});
                    s *= (1/arma::norm(s,2));
                } else {
                    s *= (1/sNorm);
                }
                arma::vec3 t = arma::cross(result.first,s);             //orth to both
                arma::vec3 Rs = matrix*s;                               //Rotate s to calculate angle of rotation

                result.second = atan2(arma::dot(Rs,t),arma::dot(Rs,s)); //Set angle of rotation for return

                return result;  //returns axis as vec3 and angle as double
            }

            inline arma::mat44 orthonormal44Inverse(const arma::mat44& m){
                arma::mat44 minverse = arma::eye(4,4);
                minverse.submat(0,0,2,2) = m.submat(0,0,2,2).t();
                minverse.submat(0,3,2,3) = -minverse.submat(0,0,2,2) * m.submat(0,3,2,3);
                return minverse;
            }

        }
    }
}

#endif // UTILITY_MATH_COORDINATES_H
