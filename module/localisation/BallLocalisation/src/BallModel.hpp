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

#ifndef MODULE_LOCALISATION_BALLMODEL_HPP
#define MODULE_LOCALISATION_BALLMODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "message/input/Sensors.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/coordinates.hpp"

namespace module::localisation {

    using message::input::Sensors;
    using message::support::FieldDescription;
    using utility::math::coordinates::cartesianToReciprocalSpherical;
    using utility::math::coordinates::cartesianToSpherical;

    template <typename Scalar>
    class BallModel {
    public:
        // The indicies for our vector
        static constexpr uint PX = 0;
        static constexpr uint PY = 1;

        static constexpr size_t size = 2;

        using StateVec = Eigen::Matrix<Scalar, size, 1>;
        using StateMat = Eigen::Matrix<Scalar, size, size>;

        struct MeasurementType {
            struct BALL {};
        };

        // Local variables for this model. Set their values from config file
        // Number of reset particles
        int n_rogues = 0;

        // Number of particles
        int n_particles = 100;
        // Range of reset particles
        Eigen::Matrix<Scalar, 2, 1> resetRange;
        // Diagonal noise matrix
        Eigen::Matrix<Scalar, 2, 1> processNoiseDiagonal;

        BallModel() : n_rogues(10), resetRange(10, 10), processNoiseDiagonal() {}

        [[nodiscard]] StateVec time(const StateVec& state, const Scalar& /*deltaT*/) const {
            return state;
        }

        [[nodiscard]] Eigen::Matrix<Scalar, 3, 1> predict(const StateVec& state,
                                                          const message::support::FieldDescription& field,
                                                          const Eigen::Matrix<Scalar, 4, 4>& Hcw) const {

            const Eigen::Matrix<Scalar, 3, 1> rBWw(state[PX], state[PY], field.ball_radius);
            const Eigen::Matrix<Scalar, 3, 1> rBCc(Eigen::Isometry3d(Hcw) * rBWw);

            return cartesianToReciprocalSpherical(rBCc);
        }

        [[nodiscard]] StateVec limit(const StateVec& state) const {
            return state;
        }

        StateMat noise(const Scalar& deltaT) {
            return processNoiseDiagonal.asDiagonal() * deltaT;
        }

        template <typename T, typename U>
        static auto difference(const T& a, const U& b) {
            return a - b;
        }

        // Getters
        [[nodiscard]] inline int getRogueCount() const {
            return n_rogues;
        }
        [[nodiscard]] StateVec get_rogue() const {
            return resetRange;
        }

        [[nodiscard]] int getParticleCount() const {
            return n_particles;
        }
    };
}  // namespace module::localisation
#endif  // MODULE_LOCALISATION_BALLMODEL_HPP
