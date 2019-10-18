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
                Eigen::Quaternion<Scalar> qGyro;
                qGyro.vec() = state.template segment<3>(WX) * deltaT * Scalar(0.5);
                qGyro.w()   = Scalar(1.0) - Scalar(0.5) * qGyro.vec().squaredNorm();
                qGyro       = Rwt * qGyro;

                newState(QW)                     = qGyro.w();
                newState.template segment<3>(QX) = qGyro.vec();

                return newState;
            }

            Eigen::Matrix<Scalar, 3, 1> predict(const StateVec& state, const MeasurementType::ACCELEROMETER&) {

                // Extract our rotation quaternion
                Eigen::Matrix<Scalar, 3, 3> Rtw =
                    Eigen::Quaternion<Scalar>(state.template segment<4>(QX)).toRotationMatrix().transpose();

                // Make a world gravity vector and rotate it into torso space
                // Where is world gravity with respest to robot orientation?
                // Multiplying a matrix (0, 0, G) is equivalent to taking the
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

            template <typename T, typename U>
            auto difference(const T& a, const U& b) {
                return a - b;
            }

            StateVec limit(const StateVec& state) {
                StateVec newState                = state;
                newState.template segment<4>(QX) = newState.template segment<4>(QX).normalized();
                return newState;
            }

            StateMat noise(const Scalar&) {
                // Return our process noise matrix
                return process_noise.asDiagonal();
            }
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module

#endif  // MODULE_PLATFORM_DARWIN_MOTIONMODEL_H
