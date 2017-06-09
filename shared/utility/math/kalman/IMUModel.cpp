/*
 * Should produce world to robot coordinate transform
 * This file is part of the Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "IMUModel.h"

namespace utility {
    namespace math {
        namespace kalman {

            Eigen::Matrix<double, IMUModel::size, 1> IMUModel::limitState(const Eigen::Matrix<double, size, 1>& state) {
                Eigen::Matrix<double, size, 1> newState = state;
                newState.rows(QW, QZ) = newState.rows(QW, QZ).normalize();
                return newState;
            }

            // @brief The process equation is used to update the systems state using the process euquations of the system.
            // @param sigma_point The sigma point representing a system state.
            // @param deltaT The amount of time that has passed since the previous update, in seconds.
            // @param measurement The reading from the rate gyroscope in rad/s used to update the orientation.
            // @return The new estimated system state.
            Eigen::Matrix<double, IMUModel::size, 1> IMUModel::timeUpdate(const Eigen::Matrix<double, size, 1>& state, double deltaT) {

                Eigen::Matrix<double, IMUModel::size, 1> newState;

                newState = state;

                //make a rotation quaternion
                const double omega = state.rows(VX, VZ).norm() + 0.00000000001;
                //Negate to compensate for some later mistake.
                //deltaT has been negative for a while and has masked an incorrect hack below
                const double theta = -omega*deltaT*0.5;
                const double sinTheta = sin(theta);
                const double cosTheta = cos(theta);
                Eigen::VectorXd vq({cosTheta,state(VX)*sinTheta/omega,state(VY)*sinTheta/omega,state(VZ)*sinTheta/omega});
                //calculate quaternion multiplication
                //TODO replace with quaternion class
                Eigen::VectorXd qcross = vq.rows(1,3).cross(state.rows(QX,QZ));
                newState(QW) = vq(0)*state(QW) - vq.rows(1,3).dot(state.rows(QX,QZ));
                newState(QX) = vq(0)*state(QX) + state(QW)*vq(1) + qcross(0);
                newState(QY) = vq(0)*state(QY) + state(QW)*vq(2) + qcross(1);
                newState(QZ) = vq(0)*state(QZ) + state(QW)*vq(3) + qcross(2);

                return newState;

            }

            // Up vector
            Eigen::Vector3d IMUModel::predictedObservation(const Eigen::Matrix<double, size, 1>& state, const MeasurementType::UP&) {

                Eigen::Vector3d up = { 2 * state[QX] * state[QZ] + 2 * state[QY] * state[QW]
                                  , 2 * state[QY] * state[QZ] - 2 * state[QX] * state[QW]
                                  , 1 - 2 * state[QX] * state[QX] - 2 * state[QY] * state[QY] };

                return up;
            }

            // Accelerometer
            Eigen::Vector3d IMUModel::predictedObservation(const Eigen::Matrix<double, size, 1>& state, const MeasurementType::ACCELEROMETER&) {

                Eigen::Vector3d down = { 2 * state[QX] * state[QZ] + 2 * state[QY] * state[QW]
                                  , 2 * state[QY] * state[QZ] - 2 * state[QX] * state[QW]
                                  , 1 - 2 * state[QX] * state[QX] - 2 * state[QY] * state[QY] };

                down *= G;

                return down;
            }

            // Gyroscope
            Eigen::Vector3d IMUModel::predictedObservation(const Eigen::Matrix<double, size, 1>& state, const MeasurementType::GYROSCOPE&) {
                return state.rows(VX, VZ);
            }


            // Forward Vector
            Eigen::Vector3d IMUModel::predictedObservation(const Eigen::Matrix<double, size, 1>& state) {

                return { 1 - 2 * state[QY] * state[QY] - 2 * state[QZ] * state[QZ]
                       , 2 * state[QX] * state[QY] + 2 * state[QZ] * state[QW]
                       , 2 * state[QX] * state[QZ] - 2 * state[QY] * state[QW] };
            }

            Eigen::VectorXd IMUModel::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;
            }

            Eigen::Matrix<double, IMUModel::size, IMUModel::size> IMUModel::processNoise() {
                return arma::diagmat(processNoiseDiagonal);
            }

        }
    }
}
