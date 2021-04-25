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

#ifndef MODULE_PLATFORM_DARWIN_MOTIONMODEL_HPP
#define MODULE_PLATFORM_DARWIN_MOTIONMODEL_HPP

/* Motion model Motion Unit*/
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utility/math/quaternion.hpp"

namespace module::platform::darwin {

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
        struct StateVec {

            // Our position in global space
            Eigen::Matrix<Scalar, 3, 1> rTWw;

            // Our velocity in global space
            Eigen::Matrix<Scalar, 3, 1> vTw;

            // Our orientation from robot to world
            Eigen::Quaternion<Scalar> Rwt;

            // Our rotational velocity in torso space
            // Gyroscope measures the angular velocity of the torso in torso space
            Eigen::Matrix<Scalar, 3, 1> omegaTTt;

            // Gyroscope Bias
            Eigen::Matrix<Scalar, 3, 1> omegaTTt_bias;

            static constexpr size_t size = 16;

            constexpr static size_t getSize() {
                return size;
            }

            enum Values {
                // rTWw
                PX = 0,
                PY = 1,
                PZ = 2,

                // vTw
                VX = 3,
                VY = 4,
                VZ = 5,

                // Rwt
                QW = 6,
                QX = 7,
                QY = 8,
                QZ = 9,

                // omegaTTt
                WX = 10,
                WY = 11,
                WZ = 12,

                // omegaTTt_bias
                BX = 13,
                BY = 14,
                BZ = 15,
            };

            // Default constructor initialises all vectors to zero, and the quaternion to the identity rotation
            StateVec()
                : rTWw(Eigen::Matrix<Scalar, 3, 1>::Zero())
                , vTw(Eigen::Matrix<Scalar, 3, 1>::Zero())
                , Rwt({1, 0, 0, 0})
                , omegaTTt(Eigen::Matrix<Scalar, 3, 1>::Zero())
                , omegaTTt_bias(Eigen::Matrix<Scalar, 3, 1>::Zero()) {}

            // Constructor from monolithic vector representation, normalising the quaternion in the process
            StateVec(const Eigen::Matrix<Scalar, size, 1>& state)
                : rTWw(state.template segment<3>(PX))
                , vTw(state.template segment<3>(VX))
                , Rwt(Eigen::Quaternion<Scalar>(state.template segment<4>(QW)).normalized())
                , omegaTTt(state.template segment<3>(WX))
                , omegaTTt_bias(state.template segment<3>(BX)) {}

            // Converts StateVec to monolithic vector representation
            Eigen::Matrix<Scalar, size, 1> getStateVec() const {
                Eigen::Matrix<Scalar, size, 1> state = Eigen::Matrix<Scalar, size, 1>::Zero();
                state.template segment<3>(PX)        = rTWw;
                state.template segment<3>(VX)        = vTw;
                state.template segment<4>(QW)        = Rwt.coeffs();
                state.template segment<3>(WX)        = omegaTTt;
                state.template segment<3>(BX)        = omegaTTt_bias;
                return state;
            }

            // Wrapper for asDiagonal for monolithic vector representation
            Eigen::Matrix<Scalar, size, size> asDiagonal() const {
                return this->getStateVec().asDiagonal();
            }

            // Operator for implicit conversion to monolithic vector representation
            operator Eigen::Matrix<Scalar, size, 1>() const {
                return this->getStateVec();
            }
        };

        static constexpr size_t size = StateVec::getSize();

        using StateMat = Eigen::Matrix<Scalar, size, size>;

        // Our static process noise diagonal vector
        StateVec process_noise;

        // The velocity decay for x/y/z velocities (1.0 = no decay)
        Eigen::Matrix<Scalar, 3, 1> timeUpdateVelocityDecay = Eigen::Matrix<Scalar, 3, 1>::Ones();

        Eigen::Matrix<Scalar, size, 1> time(const Eigen::Matrix<Scalar, size, 1>& state, const Scalar deltaT) {

            // Prepare our new state
            StateVec newState(state);

            // ********************************
            // UPDATE LINEAR POSITION/VELOCITY
            // ********************************

            // Add our velocity to our position
            newState.rTWw += newState.vTw * deltaT;

            // add velocity decay
            newState.vTw = newState.vTw.cwiseProduct(timeUpdateVelocityDecay);

            // ********************************
            // UPDATE ANGULAR POSITION/VELOCITY
            // ********************************

            // This implements the method outlined here: https://stackoverflow.com/a/41226401/11108223

            // Our angle, which we halve for use in the trig functions
            const Scalar theta_2 = newState.omegaTTt.norm() * deltaT * 0.5;

            const Scalar sin_theta_2 = std::sin(theta_2);
            const Scalar cos_theta_2 = std::cos(theta_2);

            // Our rotation axis
            const auto normalised_omegaTTt         = newState.omegaTTt.normalized();
            const Eigen::Quaternion<Scalar> update = Eigen::Quaternion<Scalar>(cos_theta_2,
                                                                               normalised_omegaTTt.x() * sin_theta_2,
                                                                               normalised_omegaTTt.y() * sin_theta_2,
                                                                               normalised_omegaTTt.z() * sin_theta_2);
            // Add our rotation update by quaternion multiplication
            newState.Rwt = (update * newState.Rwt).normalized();

            return newState;
        }

        Eigen::Matrix<Scalar, 3, 1> predict(const Eigen::Matrix<Scalar, size, 1>& state,
                                            const MeasurementType::ACCELEROMETER&) {
            // Extract our rotation quaternion
            const Eigen::Matrix<Scalar, 3, 3> Rtw = StateVec(state).Rwt.inverse().toRotationMatrix();

            // Make a world gravity vector and rotate it into torso space
            // Where is world gravity with respect to robot orientation?
            // Multiplying a matrix with (0, 0, G) is equivalent to taking the
            // third column of the matrix and multiplying it by G
            return Rtw.template rightCols<1>() * G;
        }

        Eigen::Matrix<Scalar, 3, 1> predict(const Eigen::Matrix<Scalar, size, 1>& state,
                                            const MeasurementType::GYROSCOPE&) {
            // Add predicted gyroscope bias to our predicted gyroscope
            return StateVec(state).omegaTTt + StateVec(state).omegaTTt_bias;
        }

        Eigen::Matrix<Scalar, 3, 1> predict(const Eigen::Matrix<Scalar, size, 1>& state,
                                            const MeasurementType::FLAT_FOOT_ODOMETRY&) {
            return StateVec(state).rTWw;
        }

        Eigen::Matrix<Scalar, 4, 1> predict(const Eigen::Matrix<Scalar, size, 1>& state,
                                            const MeasurementType::FLAT_FOOT_ORIENTATION&) {
            return StateVec(state).Rwt.coeffs();
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
            // If a and b represent very similar orientations, then the real part will be very close to one and
            // the imaginary part will be very close to (0, 0, 0)
            diff.w() -= Scalar(1);

            return diff.coeffs();
        }

        Eigen::Matrix<Scalar, size, 1> limit(const Eigen::Matrix<Scalar, size, 1>& state) {
            StateVec newState(state);

            // Make sure the quaternion remains normalised
            newState.Rwt = newState.Rwt.normalized();

            return newState;
        }

        Eigen::Matrix<Scalar, size, size> noise(const Scalar& deltaT) {
            // Return our process noise matrix
            return process_noise.asDiagonal() * deltaT;
        }
    };
}  // namespace module::platform::darwin
#endif  // MODULE_PLATFORM_DARWIN_MOTIONMODEL_HPP
