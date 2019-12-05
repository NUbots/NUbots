/*
 * Should produce world to robot coordinate transform
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

#include "BallModel.h"

#include "utility/input/ServoID.h"
#include "utility/math/coordinates.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace localisation {

    using message::input::Sensors;
    using message::support::FieldDescription;
    using ServoID = utility::input::ServoID;
    using utility::math::coordinates::cartesianToSpherical;

    template <typename Scalar>
    Eigen::Matrix<Scalar, BallModel<Scalar>::size, 1> BallModel<Scalar>::time(
        const Eigen::Matrix<Scalar, BallModel<Scalar>::size, 1>& state,
        double /*deltaT*/) {
        return state;
    }

    template <typename Scalar>
    Eigen::Vector3d BallModel<Scalar>::predictedObservation(
        const Eigen::Matrix<Scalar, BallModel<Scalar>::size, 1>& state,
        const FieldDescription& field,
        const Eigen::Affine3d& Hcw) const {

        Eigen::Vector4d rBWw      = {state[PX], state[PY], field.ball_radius, 1.0};
        rBWw                      = Hcw * rBWw;
        Eigen::Vector3d rBCc_cart = {rBWw[0], rBWw[1], rBWw[2]};
        Eigen::Vector3d rBCc_sph1 = cartesianToSpherical(rBCc_cart);             // in r,theta,phi
        Eigen::Vector3d rBCc_sph2 = {rBCc_sph1[0], rBCc_sph1[1], rBCc_sph1[2]};  // in roe, theta, phi, where roe is 1/r

        return rBCc_sph2;
    }

    template <typename Scalar>
    Eigen::Matrix<Scalar, BallModel<Scalar>::size, 1> BallModel<Scalar>::limit(
        const Eigen::Matrix<Scalar, BallModel<Scalar>::size, 1>& state) const {
        // TODO: configure
        return state;
    }

    template <typename Scalar>
    Eigen::Matrix<Scalar, BallModel<Scalar>::size, BallModel<Scalar>::size> BallModel<Scalar>::noise(
        const Scalar& deltaT) {
        return processNoiseDiagonal.asDiagonal() * deltaT;
    }

}  // namespace localisation
}  // namespace module
