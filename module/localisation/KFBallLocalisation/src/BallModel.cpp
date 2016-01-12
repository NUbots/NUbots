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

#include "BallModel.h"

#include <armadillo>

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/localisation/transform.h"
#include "message/localisation/FieldObject.h"


// using message::localisation::FakeOdometry;
using utility::localisation::transform::SphericalRobotObservation;

namespace module {
namespace localisation {
namespace ball {

arma::vec::fixed<BallModel::size> BallModel::ApplyVelocity(
    const arma::vec::fixed<BallModel::size>& state, double deltaT) {
    auto result = state;

    // Apply ball velocity
    result(kX) += state(kVx) * deltaT;
    result(kY) += state(kVy) * deltaT;
    result(kVx) -= result(kVx) * cfg_.ballDragCoefficient * deltaT;
    result(kVy) -= result(kVy) * cfg_.ballDragCoefficient * deltaT;

    return result;
}

arma::vec::fixed<BallModel::size> BallModel::timeUpdate(
    const arma::vec::fixed<BallModel::size>& state, double deltaT) {

    return ApplyVelocity(state, deltaT);
}

// arma::vec::fixed<BallModel::size> BallModel::timeUpdate(
//     const arma::vec::fixed<BallModel::size>& state, double deltaT,
//     const FakeOdometry& odom) {

//     auto result = ApplyVelocity(state, deltaT);

//     // Apply robot odometry / robot position change
//     result.rows(kX, kY) -= odom.torso_displacement;

//     // Rotate ball_pos by -torso_rotation.
//     arma::mat22 rot = rotationMatrix(-odom.torso_rotation);
//     result.rows(kX, kY) = rot * result.rows(kX, kY);
//     result.rows(kVx, kVy) = rot * result.rows(kVx, kVy);

//     return result;
// }

/// Return the predicted observation of an object at the given position
arma::vec BallModel::predictedObservation(
    const arma::vec::fixed<BallModel::size>& state, double ballAngle) {

    arma::vec3 ball_pos = arma::vec3({state(kX), state(kY), cfg_.ballHeight});
    auto obs = SphericalRobotObservation({0, 0}, 0, ball_pos);
    obs(1) -= ballAngle;

    arma::vec obsVel = arma::join_cols(obs,state.rows(kVx,kVy));
    return obsVel;
}

arma::vec BallModel::observationDifference(const arma::vec& a,
                                           const arma::vec& b){
    arma::vec result = a - b;
    // result(1) = utility::math::angle::normalizeAngle(result(1));
    // result(2) = utility::math::angle::normalizeAngle(result(2));
    return result;
}

arma::vec::fixed<BallModel::size> BallModel::limitState(
    const arma::vec::fixed<BallModel::size>& state) {
    auto new_state = state;
    new_state.rows(kVx,kVy) = arma::vec({0,0});
    return new_state;
}

arma::mat::fixed<BallModel::size, BallModel::size> BallModel::processNoise() {
    arma::mat noise = arma::eye(BallModel::size, BallModel::size);

    noise(kX, kX) *= cfg_.processNoisePositionFactor;
    noise(kY, kY) *= cfg_.processNoisePositionFactor;
    noise(kVx, kVx) *= cfg_.processNoiseVelocityFactor;
    noise(kVy, kVy) *= cfg_.processNoiseVelocityFactor;

    return noise;
}

}
}
}
