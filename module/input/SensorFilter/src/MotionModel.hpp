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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_INPUT_MOTIONMODEL_HPP
#define MODULE_INPUT_MOTIONMODEL_HPP

/* Motion model Motion Unit*/
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utility/math/quaternion.hpp"

namespace module::input {

    // Gravity
    static constexpr double G = 9.80665;

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
            Eigen::Matrix<Scalar, 3, 1> rTWw = Eigen::Matrix<Scalar, 3, 1>::Zero();

            // Our velocity in global space
            Eigen::Matrix<Scalar, 3, 1> vTw = Eigen::Matrix<Scalar, 3, 1>::Zero();

            // Our orientation from robot to world
            Eigen::Quaternion<Scalar> Rwt = Eigen::Quaternion<Scalar>::Identity();

            // Our rotational velocity in torso space
            // Gyroscope measures the angular velocity of the torso in torso space
            Eigen::Matrix<Scalar, 3, 1> omegaTTt = Eigen::Matrix<Scalar, 3, 1>::Zero();

            static constexpr size_t size = 13;

            [[nodiscard]] constexpr static size_t getSize() {
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
                QX = 6,
                QY = 7,
                QZ = 8,
                QW = 9,

                // omegaTTt
                WX = 10,
                WY = 11,
                WZ = 12,
            };

            // Default constructor initialises all vectors to zero, and the quaternion to the identity rotation
            StateVec() = default;

            // Constructor from monolithic vector representation, normalising the quaternion in the process
            template <typename OtherDerived>
            StateVec(const Eigen::MatrixBase<OtherDerived>& state)
                : rTWw(state.template segment<3>(PX))
                , vTw(state.template segment<3>(VX))
                , Rwt(Eigen::Quaternion<Scalar>(state.template segment<4>(QX)).normalized())
                , omegaTTt(state.template segment<3>(WX)) {}

            // Converts StateVec to monolithic vector representation
            [[nodiscard]] Eigen::Matrix<Scalar, size, 1> getStateVec() const {
                Eigen::Matrix<Scalar, size, 1> state = Eigen::Matrix<Scalar, size, 1>::Zero();
                state.template segment<3>(PX)        = rTWw;
                state.template segment<3>(VX)        = vTw;
                state.template segment<4>(QX)        = Rwt.coeffs();
                state.template segment<3>(WX)        = omegaTTt;
                return state;
            }

            // Wrapper for asDiagonal for monolithic vector representation
            [[nodiscard]] Eigen::Matrix<Scalar, size, size> asDiagonal() const {
                return getStateVec().asDiagonal();
            }

            // Operator for implicit conversion to monolithic vector representation
            [[nodiscard]] operator Eigen::Matrix<Scalar, size, 1>() const {
                return getStateVec();
            }
        };

        static constexpr size_t size = StateVec::getSize();

        using StateMat = Eigen::Matrix<Scalar, size, size>;

        // Our static process noise diagonal vector
        StateVec process_noise{};

        // The velocity decay for x/y/z velocities (1.0 = no decay)
        Eigen::Matrix<Scalar, 3, 1> timeUpdateVelocityDecay = Eigen::Matrix<Scalar, 3, 1>::Ones();

        [[nodiscard]] Eigen::Matrix<Scalar, size, 1> time(const StateVec& state, const Scalar deltaT) const {

            // Prepare our new state
            StateVec newState(state);

            // ********************************
            // UPDATE ANGULAR POSITION/VELOCITY
            // ********************************

            // Apply our rotational velocity to our orientation
            // If the norm of the gyroscope update is 0 then there is not update needed
            const Scalar norm = newState.omegaTTt.norm();
            if (norm != Scalar(0)) {
                // The gyroscope has measured a rotation of norm * deltaT around the axis
                // omegaTTt / norm
                Eigen::AngleAxis<Scalar> dq(norm * deltaT, newState.omegaTTt / norm);

                // Update our orientation
                newState.Rwt = newState.Rwt * dq;
            }

            // ********************************
            // UPDATE LINEAR POSITION/VELOCITY
            // ********************************

            // Add our velocity to our position
            newState.rTWw += newState.vTw * deltaT;

            // add velocity decay
            newState.vTw = newState.vTw.cwiseProduct(timeUpdateVelocityDecay);

            return newState;
        }

        [[nodiscard]] static Eigen::Matrix<Scalar, 3, 1> predict(
            const StateVec& state,
            const MeasurementType::ACCELEROMETER& /* acc_indicator */) {

            // Rotate world gravity vector into torso space using quaternion conjugation
            //
            // p' = q * p * q.conjugate()
            //
            // Substitute q with Rtw and p with G
            // Where G is a quaternion with real part zero and vector part (0, 0, G)
            //
            // G' = Rtw * G * Rtw.conjugate()
            //    = Rwt.conjugate() * G * Rwt.conjugate().conjugate()
            //    = Rwt.conjugate() * G * Rwt
            //
            // The vector part of G' is the result of rotating G by Rtw
            return (state.Rwt.conjugate()
                    * Eigen::Quaternion<Scalar>(Eigen::Matrix<Scalar, 4, 1>(Scalar(0), Scalar(0), Scalar(G), Scalar(0)))
                    * state.Rwt)
                .vec();
        }

        [[nodiscard]] static Eigen::Matrix<Scalar, 3, 1> predict(
            const StateVec& state,
            const MeasurementType::GYROSCOPE& /* gyro_indicator */) {
            return state.omegaTTt;
        }

        [[nodiscard]] static Eigen::Matrix<Scalar, 3, 1> predict(
            const StateVec& state,
            const MeasurementType::FLAT_FOOT_ODOMETRY& /* ff_odometry_indicator */) {
            return state.rTWw;
        }

        [[nodiscard]] static Eigen::Matrix<Scalar, 4, 1> predict(
            const StateVec& state,
            const MeasurementType::FLAT_FOOT_ORIENTATION& /* ff_orientation_indicator */) {
            return state.Rwt.coeffs();
        }

        // This function is called to determine the difference between position, velocity, and acceleration
        // measurements/predictions
        [[nodiscard]] static Eigen::Matrix<Scalar, 3, 1> difference(const Eigen::Matrix<Scalar, 3, 1>& a,
                                                                    const Eigen::Matrix<Scalar, 3, 1>& b) {
            return a - b;
        }

        // This function is called to determine the difference between quaternion measurements/predictions
        [[nodiscard]] static Eigen::Matrix<Scalar, 4, 1> difference(const Eigen::Matrix<Scalar, 4, 1>& a,
                                                                    const Eigen::Matrix<Scalar, 4, 1>& b) {
            // Find the rotation needed to get from orientation a to orientation b
            const Eigen::Quaternion<Scalar> q0(a);
            const Eigen::Quaternion<Scalar> q1(b);

            return utility::math::quaternion::difference(q0, q1).coeffs();
        }

        [[nodiscard]] static StateVec limit(const StateVec& state) {
            StateVec newState(state);

            // Make sure the quaternion remains normalised
            newState.Rwt = newState.Rwt.normalized();

            // Make sure the real part of the quaternion remains non-negative
            if (newState.Rwt.w() < Scalar(0)) {
                newState.Rwt.w() *= Scalar(-1);
                newState.Rwt.vec() *= Scalar(-1);
            }
            return newState;
        }

        [[nodiscard]] Eigen::Matrix<Scalar, size, size> noise(const Scalar& deltaT) {
            // Return our process noise matrix
            return process_noise.asDiagonal() * deltaT;
        }
    };
}  // namespace module::input
#endif  // MODULE_INPUT_MOTIONMODEL_HPP
