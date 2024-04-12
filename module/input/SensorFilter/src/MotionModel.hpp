/*
 * MIT License
 *
 * Copyright (c) 2016 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
        struct FLAT_FOOT_TRANSLATION {};
    }  // namespace MeasurementType

    template <typename Scalar>
    class MotionModel {
    public:
        struct StateVec {

            // Torso position in world space
            Eigen::Matrix<Scalar, 3, 1> rTWw = Eigen::Matrix<Scalar, 3, 1>::Zero();

            // Torso velocity in world space
            Eigen::Matrix<Scalar, 3, 1> vTw = Eigen::Matrix<Scalar, 3, 1>::Zero();

            static constexpr size_t size = 6;

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
            };

            // Default constructor initialises all vectors to zero, and the quaternion to the identity rotation
            StateVec() = default;

            // Constructor from monolithic vector representation, normalising the quaternion in the process
            template <typename OtherDerived>
            StateVec(const Eigen::MatrixBase<OtherDerived>& state)
                : rTWw(state.template segment<3>(PX)), vTw(state.template segment<3>(VX)) {}

            // Converts StateVec to monolithic vector representation
            [[nodiscard]] Eigen::Matrix<Scalar, size, 1> getStateVec() const {
                Eigen::Matrix<Scalar, size, 1> state = Eigen::Matrix<Scalar, size, 1>::Zero();
                state.template segment<3>(PX)        = rTWw;
                state.template segment<3>(VX)        = vTw;
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

        [[nodiscard]] Eigen::Matrix<Scalar, size, 1> time(const StateVec& state, const Scalar dt) const {

            // Prepare our new state
            StateVec new_state(state);

            // Add our velocity to our position
            new_state.rTWw += new_state.vTw * dt;

            return new_state;
        }

        [[nodiscard]] static Eigen::Matrix<Scalar, 3, 1> predict(
            const StateVec& state,
            const MeasurementType::FLAT_FOOT_TRANSLATION& /* ff_odometry_indicator */) {
            return state.rTWw;
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
            return state;
        }

        [[nodiscard]] Eigen::Matrix<Scalar, size, size> noise(const Scalar& dt) {
            // Return our process noise matrix
            return process_noise.asDiagonal() * dt;
        }
    };
}  // namespace module::input
#endif  // MODULE_INPUT_MOTIONMODEL_HPP
