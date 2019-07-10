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
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace localisation {

    using message::input::Sensors;
    using message::support::FieldDescription;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using ServoID = utility::input::ServoID;
    using utility::math::coordinates::cartesianToSpherical;

    arma::vec::fixed<BallModel::size> BallModel::timeUpdate(const arma::vec::fixed<size>& state, double /*deltaT*/) {
        return state;
    }

    arma::vec3 BallModel::predictedObservation(const arma::vec::fixed<size>& state,
                                               const FieldDescription& field,
                                               const Transform3D& Hcw) const {

        arma::vec3 rBWw      = {state[PX], state[PY], field.ball_radius};
        arma::vec3 rBCc_cart = Hcw.transformPoint(rBWw);
        arma::vec3 rBCc_sph1 = cartesianToSpherical(rBCc_cart);             // in r,theta,phi
        arma::vec3 rBCc_sph2 = {rBCc_sph1[0], rBCc_sph1[1], rBCc_sph1[2]};  // in roe, theta, phi, where roe is 1/r

        return rBCc_sph2;
    }

    arma::vec BallModel::observationDifference(const arma::vec& measurement, const arma::vec3& rBCc) const {

        return measurement - rBCc;  // not sure this raw delta vector will be what the UKF needs
    }

    arma::vec::fixed<BallModel::size> BallModel::limitState(const arma::vec::fixed<size>& state) const {
        // TODO: configure
        return state;
    }

    arma::mat::fixed<BallModel::size, BallModel::size> BallModel::processNoise() const {
        return arma::diagmat(processNoiseDiagonal);
    }
}  // namespace localisation
}  // namespace module
