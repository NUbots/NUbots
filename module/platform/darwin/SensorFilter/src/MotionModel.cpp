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

            // ********************************
            // UPDATE LINEAR POSITION/VELOCITY
            // ********************************

            // Add our velocity to our position
            newState.rows(PX, PZ) += state.rows(VX, VZ) * deltaT;

            // add velocity decay
            newState.rows(VX, VZ) = newState.rows(VX, VZ) % timeUpdateVelocityDecay;


            // ********************************
            // UPDATE ANGULAR POSITION/VELOCITY
            // ********************************

            // Extract our unit quaternion rotation
            UnitQuaternion Rwt(state.rows(QW, QZ));

            // Apply our rotational velocity to our orientation
            double t_2 = deltaT * 0.5;
            UnitQuaternion qGyro;
            qGyro.imaginary() = state.rows(WX, WZ) * t_2;
            qGyro.real()      = 1.0 - 0.5 * arma::sum(arma::square(qGyro.imaginary()));

            newState.rows(QW, QZ) = Rwt * qGyro;

            return newState;
        }

        // Accelerometer
        arma::vec3 MotionModel::predictedObservation(const arma::vec::fixed<size>& state,
                                                     const MeasurementType::ACCELEROMETER&) {

            // Extract our rotation quaternion
            UnitQuaternion Rwt(state.rows(QW, QZ));

            // Make a world gravity vector and rotate it into torso space
            // Where is world gravity with respest to robot orientation?
            return Rwt.i().rotateVector(arma::vec3({0, 0, G}));
        }

        // Gyroscope
        arma::vec3 MotionModel::predictedObservation(const arma::vec::fixed<size>& state,
                                                     const MeasurementType::GYROSCOPE&) {
            // Add predicted gyroscope bias to our predicted gyroscope
            return state.rows(WX, WZ) + state.rows(BX, BZ);
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
