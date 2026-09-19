/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#ifndef MODULE_LOCALISATION_BALLMODEL_HPP
#define MODULE_LOCALISATION_BALLMODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>

namespace module::localisation {

    namespace MeasurementType {
        struct BALL_POSITION {};
    }  // namespace MeasurementType

    template <typename Scalar>
    class BallModel {
    public:
        struct StateVec {

            // Ball position in world {w} space (x, y)
            Eigen::Matrix<Scalar, 2, 1> rBWw = Eigen::Matrix<Scalar, 2, 1>::Zero();

            // Ball velocity in world {w} space (dx, dy)
            Eigen::Matrix<Scalar, 2, 1> vBw = Eigen::Matrix<Scalar, 2, 1>::Zero();

            static constexpr size_t size = 4;

            [[nodiscard]] constexpr static size_t getSize() {
                return size;
            }

            enum Values {
                // rBWw
                PX = 0,
                PY = 1,

                // vBw
                VX = 2,
                VY = 3,
            };

            StateVec() = default;

            template <typename OtherDerived>
            StateVec(const Eigen::MatrixBase<OtherDerived>& state)
                : rBWw(state.template segment<2>(PX)), vBw(state.template segment<2>(VX)) {}

            [[nodiscard]] Eigen::Matrix<Scalar, size, 1> getStateVec() const {
                Eigen::Matrix<Scalar, size, 1> state = Eigen::Matrix<Scalar, size, 1>::Zero();
                state.template segment<2>(PX)        = rBWw;
                state.template segment<2>(VX)        = vBw;
                return state;
            }

            [[nodiscard]] Eigen::Matrix<Scalar, size, size> asDiagonal() const {
                return getStateVec().asDiagonal();
            }

            [[nodiscard]] operator Eigen::Matrix<Scalar, size, 1>() const {
                return getStateVec();
            }
        };

        static constexpr size_t size = StateVec::getSize();

        using StateMat = Eigen::Matrix<Scalar, size, size>;

        /// White-noise-acceleration spectral density (m^2/s^3): how quickly the ball's velocity may change
        /// between updates. Kicks are handled by the filter's association logic, not by this noise.
        Scalar acceleration_noise = Scalar(1);

        [[nodiscard]] Eigen::Matrix<Scalar, size, 1> time(const StateVec& state, const Scalar delta_T) const {
            StateVec new_state(state);

            // Constant velocity. Rolling deceleration is applied to the mean by BallLocalisation, not here. The sigma
            // points sit very close to the mean (alpha 0.1), so pushing them through the deceleration pulls all of a
            // slow ball's points towards zero speed. That shrinks the velocity variance every step and leaves the
            // filter too sure a resting ball will stay still.
            new_state.rBWw += state.vBw * delta_T;

            return new_state;
        }

        [[nodiscard]] static Eigen::Matrix<Scalar, 2, 1> predict(
            const StateVec& state,
            const MeasurementType::BALL_POSITION& /* position_indicator */) {
            return state.rBWw;
        }

        [[nodiscard]] static Eigen::Matrix<Scalar, 2, 1> difference(const Eigen::Matrix<Scalar, 2, 1>& a,
                                                                    const Eigen::Matrix<Scalar, 2, 1>& b) {
            return a - b;
        }

        [[nodiscard]] static StateVec limit(const StateVec& state) {
            return state;
        }

        /// Discrete white-noise-acceleration process noise: per axis q * [[dt^3/3, dt^2/2], [dt^2/2, dt]]
        [[nodiscard]] Eigen::Matrix<Scalar, size, size> noise(const Scalar& deltaT) {
            const Scalar q                      = acceleration_noise;
            const Scalar t2                     = deltaT * deltaT;
            Eigen::Matrix<Scalar, size, size> Q = Eigen::Matrix<Scalar, size, size>::Zero();
            for (int axis = 0; axis < 2; ++axis) {
                const int p = StateVec::PX + axis;
                const int v = StateVec::VX + axis;
                Q(p, p)     = q * t2 * deltaT / Scalar(3);
                Q(p, v)     = q * t2 / Scalar(2);
                Q(v, p)     = q * t2 / Scalar(2);
                Q(v, v)     = q * deltaT;
            }
            return Q;
        }
    };
}  // namespace module::localisation
#endif  // MODULE_LOCALISATION_BALLMODEL_HPP
