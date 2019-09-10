/*
 * Should produce world to robot coordinate transform
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

#include "MotionModel.h"

namespace module {
namespace platform {
    namespace darwin {

        Eigen::Matrix<double, MotionModel::size, 1> MotionModel::limitState(
            const Eigen::Matrix<double, MotionModel::size, 1>& state) {
            Eigen::Matrix<double, MotionModel::size, 1> newState = state;
            newState.segment<QW - QX + 1>(QX)                    = newState.segment<QW - QX + 1>(QX).normalized();
            return newState;
        }

        // @brief The process equation is used to update the systems state using the process euquations of the system.
        // @param sigma_point The sigma point representing a system state.
        // @param deltaT The amount of time that has passed since the previous update, in seconds.
        // @param measurement The reading from the rate gyroscope in rad/s used to update the orientation.
        // @return The new estimated system state.
        Eigen::Matrix<double, MotionModel::size, 1> MotionModel::timeUpdate(
            const Eigen::Matrix<double, MotionModel::size, 1>& state,
            double deltaT) {

            // Prepare our new state
            Eigen::Matrix<double, MotionModel::size, 1> newState = state;

            // ********************************
            // UPDATE LINEAR POSITION/VELOCITY
            // ********************************

            // Add our velocity to our position
            newState.segment<PZ - PX + 1>(PX) += state.segment<VZ - VX + 1>(VX) * deltaT;

            // add velocity decay
            newState.segment<VZ - VX + 1>(VX) = newState.segment<VZ - VX + 1>(VX).cwiseProduct(timeUpdateVelocityDecay);


            // ********************************
            // UPDATE ANGULAR POSITION/VELOCITY
            // ********************************

            // Extract our unit quaternion rotation
            Eigen::Quaterniond Rwt(state.segment<QW - QX + 1>(QX));

            // Apply our rotational velocity to our orientation
            double t_2 = deltaT * 0.5;
            Eigen::Quaterniond qGyro;
            qGyro.vec() = state.segment<WZ - WX + 1>(WX) * t_2;
            qGyro.w()   = 1.0 - 0.5 * qGyro.vec().squaredNorm();

            newState.segment<QW - QX + 1>(QX) = Rwt * qGyro;

            return newState;
        }

        // Accelerometer
        Eigen::Vector3d MotionModel::predictedObservation(const Eigen::Matrix<double, MotionModel::size, 1>& state,
                                                          const MeasurementType::ACCELEROMETER&) {

            // Extract our rotation quaternion
            Eigen::Quaterniond Rwt(state.segment<QW - QX + 1>(QX));

            // Make a world gravity vector and rotate it into torso space
            // Where is world gravity with respest to robot orientation?
            return Rwt.i().rotateVector(Eigen::Vector3d(0.0, 0.0, G));
        }

        // Gyroscope
        Eigen::Vector3d MotionModel::predictedObservation(const Eigen::Matrix<double, MotionModel::size, 1>& state,
                                                          const MeasurementType::GYROSCOPE&) {
            // Add predicted gyroscope bias to our predicted gyroscope
            return state.segment<WZ - WX + 1>(WX) + state.segment<BZ - BX + 1>(BX);
        }

        Eigen::Vector3d MotionModel::predictedObservation(const Eigen::Matrix<double, MotionModel::size, 1>& state,
                                                          const MeasurementType::FLAT_FOOT_ODOMETRY&) {
            return state.segment<PZ - PX + 1>(PX);
        }

        Eigen::Vector4d MotionModel::predictedObservation(const Eigen::Matrix<double, MotionModel::size, 1>& state,
                                                          const MeasurementType::FLAT_FOOT_ORIENTATION&) {
            return state.segment<QW - QX + 1>(QX);
        }

        template <int N>
        Eigen::Matrix<double, N, 1> MotionModel::observationDifference(const Eigen::Matrix<double, N, 1>& a,
                                                                       const Eigen::Matrix<double, N, 1>& b) {
            return a - b;
        }

        const Eigen::Matrix<double, MotionModel::size, MotionModel::size>& MotionModel::processNoise() {
            // Return our process noise matrix
            return processNoiseMatrix;
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace module
