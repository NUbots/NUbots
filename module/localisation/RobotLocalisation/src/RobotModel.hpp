/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#ifndef MODULE_LOCALISATION_ROBOTMODEL_HPP
#define MODULE_LOCALISATION_ROBOTMODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace module::localisation {

    namespace MeasurementType {
        struct ROBOT_POSITION {};
    }  // namespace MeasurementType

    template <typename Scalar>
    class RobotModel {
    public:
        struct StateVec {

            // Robot position in world {w} space (x, y)
            Eigen::Matrix<Scalar, 2, 1> rRWw = Eigen::Matrix<Scalar, 2, 1>::Zero();

            // Robot velocity in world {w} space (dx, dy)
            Eigen::Matrix<Scalar, 2, 1> vRw = Eigen::Matrix<Scalar, 2, 1>::Zero();

            static constexpr size_t size = 4;

            [[nodiscard]] constexpr static size_t getSize() {
                return size;
            }

            enum Values {
                // rRWw
                PX = 0,
                PY = 1,

                // vRw
                VX = 2,
                VY = 3,
            };

            StateVec() = default;

            template <typename OtherDerived>
            StateVec(const Eigen::MatrixBase<OtherDerived>& state)
                : rRWw(state.template segment<2>(PX)), vRw(state.template segment<2>(VX)) {}

            [[nodiscard]] Eigen::Matrix<Scalar, size, 1> getStateVec() const {
                Eigen::Matrix<Scalar, size, 1> state = Eigen::Matrix<Scalar, size, 1>::Zero();
                state.template segment<2>(PX)        = rRWw;
                state.template segment<2>(VX)        = vRw;
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

        StateVec process_noise{};

        [[nodiscard]] Eigen::Matrix<Scalar, size, 1> time(const StateVec& state, const Scalar dt) const {

            StateVec new_state(state);

            // Update position based on velocity
            new_state.rRWw += new_state.vRw * dt;

            return new_state;
        }

        [[nodiscard]] static Eigen::Matrix<Scalar, 2, 1> predict(
            const StateVec& state,
            const MeasurementType::ROBOT_POSITION& /* position_indicator */) {
            return state.rRWw;
        }

        [[nodiscard]] static Eigen::Matrix<Scalar, 2, 1> difference(const Eigen::Matrix<Scalar, 2, 1>& a,
                                                                    const Eigen::Matrix<Scalar, 2, 1>& b) {
            return a - b;
        }

        [[nodiscard]] static StateVec limit(const StateVec& state) {
            return state;
        }

        [[nodiscard]] Eigen::Matrix<Scalar, size, size> noise(const Scalar& deltaT) {
            return process_noise.asDiagonal() * deltaT;
        }
    };
}  // namespace module::localisation
#endif  // MODULE_LOCALISATION_ROBOTMODEL_HPP
