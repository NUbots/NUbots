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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_PLATFORM_DARWIN_MOTIONMODEL_H
#define MODULE_PLATFORM_DARWIN_MOTIONMODEL_H

/* Motion model Motion Unit*/
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utility/math/quaternion.h"

namespace module {
namespace platform {
    namespace darwin {

        // Gravity
        static constexpr double G = -9.80665;

        namespace MeasurementType {
            struct GYROSCOPE {};
            struct ACCELEROMETER {};
            struct FLAT_FOOT_ODOMETRY {};
            struct FLAT_FOOT_ORIENTATION {};
        }  // namespace MeasurementType

        template <typename Scalar>
        class MotionModel {
        public:
            enum Values {
                // Our position in global space
                // rTWw
                PX = 0,
                PY = 1,
                PZ = 2,

                // Our velocity in global space
                // vTw
                VX = 3,
                VY = 4,
                VZ = 5,

                // Our orientation from robot to world
                // Rwt
                QX = 6,
                QY = 7,
                QZ = 8,
                QW = 9,

                // Our rotational velocity in torso space
                // Gyroscope measures the angular velocity of the torso in torso space
                // omegaTTt
                WX = 10,
                WY = 11,
                WZ = 12,

                // Gyroscope Bias
                // omegaTTt
                BX = 13,
                BY = 14,
                BZ = 15,
            };

            // The size of our state
            static constexpr size_t size = 16;

            using StateVec = Eigen::Matrix<Scalar, size, 1>;
            using StateMat = Eigen::Matrix<Scalar, size, size>;

            // Our static process noise diagonal vector
            StateVec process_noise;

            // The velocity decay for x/y/z velocities (1.0 = no decay)
            Eigen::Matrix<Scalar, 3, 1> timeUpdateVelocityDecay = Eigen::Matrix<Scalar, 3, 1>::Ones();

            StateVec time(const StateVec& state, Scalar deltaT) {

                // Prepare our new state
                StateVec newState = state;

                // ********************************
                // UPDATE LINEAR POSITION/VELOCITY
                // ********************************

                // Add our velocity to our position
                newState.template segment<3>(PX) += state.template segment<3>(VX) * deltaT;

                // add velocity decay
                newState.template segment<3>(VX) =
                    newState.template segment<3>(VX).cwiseProduct(timeUpdateVelocityDecay);

                // ********************************
                // UPDATE ANGULAR POSITION/VELOCITY
                // ********************************

                // Extract our unit quaternion rotation
                Eigen::Quaternion<Scalar> Rwt(state.template segment<4>(QX));

                // Apply our rotational velocity to our orientation
                // https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
                // Quaternions are stored internally as (x, y, z, w)
                const Scalar t_2 = deltaT * Scalar(0.5);
                newState.template segment<4>(QX) =
                    Rwt.coeffs()
                    + t_2 * (Eigen::Quaternion<Scalar>(0.0, state[WX], state[WY], state[WZ]) * Rwt).coeffs();

                return newState;
            }

            Eigen::Matrix<Scalar, 3, 1> predict(const StateVec& state, const MeasurementType::ACCELEROMETER&) {
                // Extract our rotation quaternion
                Eigen::Matrix<Scalar, 3, 3> Rtw =
                    Eigen::Quaternion<Scalar>(state.template segment<4>(QX)).toRotationMatrix().transpose();

                // Make a world gravity vector and rotate it into torso space
                // Where is world gravity with respest to robot orientation?
                // Multiplying a matrix with (0, 0, G) is equivalent to taking the
                // third column of the matrix and multiplying it by G
                return Rtw.template rightCols<1>() * G;
            }

            Eigen::Matrix<Scalar, 3, 1> predict(const StateVec& state, const MeasurementType::GYROSCOPE&) {
                // Add predicted gyroscope bias to our predicted gyroscope
                return state.template segment<3>(WX) + state.template segment<3>(BX);
            }

            Eigen::Matrix<Scalar, 3, 1> predict(const StateVec& state, const MeasurementType::FLAT_FOOT_ODOMETRY&) {
                return state.template segment<3>(PX);
            }

            Eigen::Matrix<Scalar, 4, 1> predict(const StateVec& state, const MeasurementType::FLAT_FOOT_ORIENTATION&) {
                return state.template segment<4>(QX);
            }

            // This function is called to determine the difference between position, velocity, and acceleration
            // measurements/predictions
            Eigen::Matrix<Scalar, 3, 1> difference(const Eigen::Matrix<Scalar, 3, 1>& a,
                                                   const Eigen::Matrix<Scalar, 3, 1>& b) {
                return a - b;
            }

            // This function is called to determine the difference between quaternion measurements/predictions
            Eigen::Matrix<Scalar, 4, 1> difference(const Eigen::Matrix<Scalar, 4, 1>& a,
                                                   const Eigen::Matrix<Scalar, 4, 1>& b) {
                // Find the rotation needed to get from orientation a to orientation b
                Eigen::Quaternion<Scalar> diff =
                    utility::math::quaternion::difference(Eigen::Quaternion<Scalar>(a), Eigen::Quaternion<Scalar>(b));

                // Take the difference with the identity rotation (1, 0, 0, 0)
                // If a and b represent very similar orientations, then the real part will be very close to one and the
                // imaginary part will be very close to (0, 0, 0)
                diff.w() -= Scalar(1);

                return diff.coeffs();
            }

            StateVec limit(const StateVec& state) {
                StateVec newState = state;

                // Make sure the quaternion remains normalised
                newState.template segment<4>(QX) =
                    Eigen::Quaternion<Scalar>(newState.template segment<4>(QX)).normalized().coeffs();

                return newState;
            }

            StateMat noise(const Scalar& deltaT) {
                // Return our process noise matrix
                return process_noise.asDiagonal() * deltaT;
            }
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module

#endif  // MODULE_PLATFORM_DARWIN_MOTIONMODEL_H
