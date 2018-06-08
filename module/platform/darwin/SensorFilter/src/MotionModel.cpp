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

#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Rotation3D.h"

namespace module {
namespace platform {
    namespace darwin {

        using utility::math::geometry::UnitQuaternion;
        using utility::math::matrix::Rotation3D;

        arma::vec::fixed<MotionModel::size> MotionModel::limitState(const arma::vec::fixed<size>& state) {
            arma::vec::fixed<size> newState = state;
            newState.rows(QW, QZ)           = arma::normalise(newState.rows(QW, QZ));
            return newState;
        }

        // @brief The process equation is used to update the systems state using the process euquations of the system.
        // @param sigma_point The sigma point representing a system state.
        // @param deltaT The amount of time that has passed since the previous update, in seconds.
        // @param measurement The reading from the rate gyroscope in rad/s used to update the orientation.
        // @return The new estimated system state.
        arma::vec::fixed<MotionModel::size> MotionModel::timeUpdate(const arma::vec::fixed<size>& state,
                                                                    double deltaT) {

            // Prepare our new state
            arma::vec::fixed<MotionModel::size> newState = state;

            // Extract our unit quaternion rotation
            UnitQuaternion rotation(state.rows(QW, QZ));

            // Add our velocity to our position
            newState.rows(PX, PZ) += state.rows(VX, VZ) * deltaT;

            // Apply our rotational velocity to our orientation
            double t_2 = deltaT * 0.5;
            UnitQuaternion qGyro;
            qGyro.imaginary() = state.rows(WX, WZ) * t_2;
            qGyro.real()      = 1.0 - 0.5 * arma::sum(arma::square(qGyro.imaginary()));

            newState.rows(QW, QZ) = qGyro * rotation;

            // add velocity decay
            newState.rows(VX, VZ) = newState.rows(VX, VZ) % timeUpdateVelocityDecay;

            return newState;
        }

        // Accelerometer
        arma::vec3 MotionModel::predictedObservation(const arma::vec::fixed<size>& state,
                                                     const MeasurementType::ACCELEROMETER&) {

            // Extract our rotation quaternion
            UnitQuaternion rotation(state.rows(QW, QZ));

            // Make a gravity vector and return it
            return rotation.rotateVector(arma::vec3({0, 0, G}));
        }

        // Gyroscope
        arma::vec3 MotionModel::predictedObservation(const arma::vec::fixed<size>& state,
                                                     const MeasurementType::GYROSCOPE&) {
            return state.rows(WX, WZ);
        }

        // Foot up with z
        arma::vec4 MotionModel::predictedObservation(const arma::vec::fixed<size>& state,
                                                     const MeasurementType::FOOT_UP_WITH_Z&) {

            arma::vec4 prediction;

            // Extract our rotation quaternion
            UnitQuaternion rotation(state.rows(QW, QZ));

            // First 3 is the up vector in torso space
            prediction.rows(0, 2) = rotation.rotateVector(arma::vec3({0, 0, 1}));

            // 4th component is our z height
            prediction[3] = state[PZ];

            return prediction;
        }

        arma::vec3 MotionModel::predictedObservation(const arma::vec::fixed<size>& state,
                                                     const MeasurementType::FLAT_FOOT_ODOMETRY&) {

            return state.rows(PX, PZ);
        }

        arma::vec4 MotionModel::predictedObservation(const arma::vec::fixed<size>& state,
                                                     const MeasurementType::FLAT_FOOT_ORIENTATION&) {

            return state.rows(QW, QZ);
        }

        arma::vec MotionModel::observationDifference(const arma::vec& a, const arma::vec& b) {
            return a - b;
        }

        const arma::mat::fixed<MotionModel::size, MotionModel::size>& MotionModel::processNoise() {
            // Return our process noise matrix
            return processNoiseMatrix;
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace module
