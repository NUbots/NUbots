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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_ROBOTMODEL_HPP
#define MODULES_LOCALISATION_ROBOTMODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>
#include <vector>

#include "message/input/Sensors.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/Goal.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/localisation/transform.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/coordinates.hpp"

namespace module::localisation {

    using message::input::Sensors;
    using message::support::FieldDescription;
    using message::vision::Goal;
    using utility::input::ServoID;
    using utility::localisation::fieldStateToTransform3D;
    using utility::math::angle::normalizeAngle;
    using utility::math::coordinates::cartesianToReciprocalSpherical;

    template <typename Scalar>
    class RobotModel {
    public:
        static constexpr size_t size = 3;

        using StateVec = Eigen::Matrix<Scalar, size, 1>;
        using StateMat = Eigen::Matrix<Scalar, size, size>;


        enum Components : int {
            // Field center in world space
            kX = 0,
            kY = 1,
            // Angle is the angle from the robot world forward direction to the field forward direction
            kAngle = 2
        };

        // Local variables for this model. Set their values from config file
        // Number of reset particles
        int n_rogues = 0;
        // Number of particles
        int n_particles = 100;
        // Range of reset particles
        Eigen::Matrix<Scalar, 3, 1> resetRange;
        // Diagonal for 3x3 noise matrix (which is diagonal)
        Eigen::Matrix<Scalar, 3, 1> processNoiseDiagonal;

        RobotModel()
            : resetRange(Eigen::Matrix<Scalar, 3, 1>::Zero())
            , processNoiseDiagonal(Eigen::Matrix<Scalar, 3, 1>::Ones()) {}

        StateVec time(const StateVec& state, double /*deltaT*/) {
            return state;
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> predict(const StateVec& state,
                                                         const Eigen::Matrix<Scalar, 3, 1>& rGFf,  // true goal position
                                                         const Eigen::Matrix<Scalar, 4, 4>& Hcw) {

            // Create a transform from the field state
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hfw;
            Hfw.translation() = Eigen::Matrix<Scalar, 3, 1>(state.x(), state.y(), 0);
            Hfw.linear() = Eigen::AngleAxis<Scalar>(state.z(), Eigen::Matrix<Scalar, 3, 1>::UnitZ()).toRotationMatrix();

            const Eigen::Transform<Scalar, 3, Eigen::Isometry> Hcf(Hcw * Hfw.inverse().matrix());

            const Eigen::Matrix<Scalar, 3, 1> rGCc(Hcf * rGFf);
            return cartesianToReciprocalSpherical(rGCc);
        }

        StateVec limit(const StateVec& state) {
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
        [[nodiscard]] int getRogueCount() const {
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
#endif
