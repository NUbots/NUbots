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

#ifndef MODULE_LOCALISATION_BALLMODEL_H
#define MODULE_LOCALISATION_BALLMODEL_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "utility/math/coordinates.h"

namespace module {
namespace localisation {

    using message::input::Sensors;
    using message::support::FieldDescription;
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

        Eigen::Vector2d processNoiseDiagonal;


        BallModel() : processNoiseDiagonal(Eigen::Vector2d::Identity()) {}  // empty constructor

        StateVec time(const StateVec& state, double deltaT) {
            return state;
        }

        Eigen::Vector3d predictedObservation(const StateVec& state,
                                             const message::support::FieldDescription& field,
                                             const Eigen::Affine3d& Hcw) const {

            Eigen::Vector4d rBWw      = {state[PX], state[PY], field.ball_radius, 1.0};
            rBWw                      = Hcw * rBWw;
            Eigen::Vector3d rBCc_cart = {rBWw[0], rBWw[1], rBWw[2]};
            Eigen::Vector3d rBCc_sph1 = cartesianToSpherical(rBCc_cart);  // in r,theta,phi
            Eigen::Vector3d rBCc_sph2 = {
                rBCc_sph1[0], rBCc_sph1[1], rBCc_sph1[2]};  // in roe, theta, phi, where roe is 1/r

            return rBCc_sph2;
        }

        StateVec limit(const StateVec& state) const {
            return state;
        }

        StateMat noise(const Scalar& deltaT) {
            return processNoiseDiagonal.asDiagonal() * deltaT;
        }

        template <typename... Args>
        Eigen::Matrix<Scalar, 1, 1> predict(const StateVec& state, const Args&... params) {
            // Our prediction is the first state
            return Eigen::Matrix<Scalar, 1, 1>(state[PX]);
        }

        template <typename T, typename U>
        static auto difference(const T& a, const U& b) {
            return a - b;
        }

        // number and range of reset particles
        int n_rogues               = 10;
        Eigen::Vector2d resetRange = {10, 10};

        // Getters
        inline int getRogueCount() const {
            return n_rogues;
        }
        Eigen::Vector2d getRogueRange() const {
            return resetRange;
        }
    };
}  // namespace localisation
}  // namespace module
#endif  // MODULE_LOCALISATION_BALLMODEL_H
