/*
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

#ifndef UTILITY_MATH_KALMAN_IMUMODEL_H
#define UTILITY_MATH_KALMAN_IMUMODEL_H

/* Inertial Motion Unit*/

namespace utility {
    namespace math {
        namespace kalman {

            class IMUModel {
            public:

                static constexpr double G = -9.80665;

                // The indicies for our vector
                static constexpr uint VX = 0;
                static constexpr uint VY = 1;
                static constexpr uint VZ = 2;
                static constexpr uint QW = 3;
                static constexpr uint QX = 4;
                static constexpr uint QY = 5;
                static constexpr uint QZ = 6;

                struct MeasurementType {
                    struct GYROSCOPE {};
                    struct ACCELEROMETER {};
                    struct FORWARD {};
                    struct UP {};
                };

                Eigen::VectorXd processNoiseDiagonal;

                static constexpr size_t size = 7;

                IMUModel() : processNoiseDiagonal() {} // empty constructor

                Eigen::Matrix<double, size, 1> timeUpdate(const Eigen::Matrix<double, size, 1>& state, double deltaT);

                Eigen::Vector3d predictedObservation(const Eigen::Matrix<double, size, 1>& state);
                Eigen::Vector3d predictedObservation(const Eigen::Matrix<double, size, 1>& state, const MeasurementType::UP&);
                Eigen::Vector3d predictedObservation(const Eigen::Matrix<double, size, 1>& state, const MeasurementType::ACCELEROMETER&);
                Eigen::Vector3d predictedObservation(const Eigen::Matrix<double, size, 1>& state, const MeasurementType::GYROSCOPE&);

                Eigen::VectorXd observationDifference(const arma::vec& a, const arma::vec& b);

                Eigen::Matrix<double, size, 1> limitState(const Eigen::Matrix<double, size, 1>& state);

                arma::mat::fixed<size, size> processNoise();
            };

        }
    }
}
#endif
