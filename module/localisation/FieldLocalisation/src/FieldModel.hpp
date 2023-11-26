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
 * Copyright 2023 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_ROBOTMODEL_HPP
#define MODULES_LOCALISATION_ROBOTMODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace module::localisation {

    template <typename Scalar>
    class FieldModel {
    public:
        /// @brief Size of the state vector (x, y, theta)
        static constexpr size_t size = 3;

        using StateVec = Eigen::Matrix<Scalar, size, 1>;
        using StateMat = Eigen::Matrix<Scalar, size, size>;

        enum Components : int {
            /// @brief World {w} x position from field {f}
            x = 0,

            /// @brief World {w} y position from field {f}
            y = 1,

            /// @brief Angle of world {w} from field {f}
            theta = 2
        };

        /// @brief Number of reset particles
        int n_rogues = 0;

        /// @brief Number of particles
        int n_particles = 100;

        /// @brief Range of reset particles
        Eigen::Matrix<Scalar, 3, 1> reset_range;

        /// @brief Diagonal for 3x3 noise matrix (which is diagonal)
        Eigen::Matrix<Scalar, 3, 1> process_noise_diagonal;

        FieldModel()
            : reset_range(Eigen::Matrix<Scalar, 3, 1>::Zero())
            , process_noise_diagonal(Eigen::Matrix<Scalar, 3, 1>::Ones()) {}

        StateVec time(const StateVec& state, double /*deltaT*/) {
            return state;
        }

        StateVec limit(const StateVec& state) {
            return state;
        }

        StateMat noise(const Scalar& deltaT) {
            return process_noise_diagonal.asDiagonal() * deltaT;
        }

        // Getters
        [[nodiscard]] int get_rogue_count() const {
            return n_rogues;
        }
        [[nodiscard]] StateVec get_rogue() const {
            return reset_range;
        }

        [[nodiscard]] int get_particle_count() const {
            return n_particles;
        }
    };
}  // namespace module::localisation
#endif
