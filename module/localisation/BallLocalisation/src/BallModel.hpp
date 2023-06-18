#ifndef MODULE_LOCALISATION_BALLMODEL_HPP
#define MODULE_LOCALISATION_BALLMODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

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

        StateVec process_noise{};

        [[nodiscard]] Eigen::Matrix<Scalar, size, 1> time(const StateVec& state, const Scalar delta_T) const {

            StateVec new_state(state);

            // Update position based on velocity
            new_state.rBWw += new_state.vBw * delta_T;

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

        [[nodiscard]] Eigen::Matrix<Scalar, size, size> noise(const Scalar& deltaT) {
            return process_noise.asDiagonal() * deltaT;
        }
    };
}  // namespace module::localisation
#endif  // MODULE_LOCALISATION_BALLMODEL_HPP
