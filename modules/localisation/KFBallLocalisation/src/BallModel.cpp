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
#include "utility/math/matrix.h"
#include "utility/math/coordinates.h"
#include "messages/localisation/FieldObject.h"

using messages::localisation::FakeOdometry;
using utility::math::matrix::rotationMatrix;

namespace modules {
namespace localisation {
namespace ball {

arma::vec::fixed<BallModel::size> BallModel::ApplyVelocity(
    const arma::vec::fixed<BallModel::size>& state, double deltaT) {
    auto result = state;

    // Apply ball velocity
    result(kX) += state(kVx) * deltaT;
    result(kY) += state(kVy) * deltaT;
    result(kVx) -= result(kVx) * ballDragCoefficient * deltaT;
    result(kVy) -= result(kVy) * ballDragCoefficient * deltaT;

    return result;
}

arma::vec::fixed<BallModel::size> BallModel::timeUpdate(
    const arma::vec::fixed<BallModel::size>& state, double deltaT) {

    return ApplyVelocity(state, deltaT);
}

arma::vec::fixed<BallModel::size> BallModel::timeUpdate(
    const arma::vec::fixed<BallModel::size>& state, double deltaT,
    const FakeOdometry& odom) {

    auto result = ApplyVelocity(state, deltaT);

    // Apply robot odometry / robot position change
    result.rows(kX, kY) -= odom.torso_displacement;

    // Rotate ball_pos by -torso_rotation.
    arma::mat22 rot = rotationMatrix(-odom.torso_rotation);
    result.rows(kX, kY) = rot * result.rows(kX, kY);
    result.rows(kVx, kVy) = rot * result.rows(kVx, kVy);

    return result;
}

/// Return the predicted observation of an object at the given position
arma::vec BallModel::predictedObservation(
    const arma::vec::fixed<BallModel::size>& state) {

    // // Robot-relative cartesian
    return { state(kX), state(kY) };

    // Distance and unit vector heading
    // arma::vec2 radial = utility::math::coordinates::Cartesian2Radial(state.rows(0, 1));
    // auto heading_angle = radial[1];
    // auto heading_x = std::cos(heading_angle);
    // auto heading_y = std::sin(heading_angle);
    // return {radial[0], heading_x, heading_y};
}

arma::vec BallModel::observationDifference(const arma::vec& a,
                                            const arma::vec& b){
    // Distance and unit vector heading
    return a - b;
}

arma::vec::fixed<BallModel::size> BallModel::limitState(
    const arma::vec::fixed<BallModel::size>& state) {

    return { state(kX), state(kY), state(kVx), state(kVy) };


    // // Radial coordinates
    // return { state[kX],
    //     utility::math::angle::normalizeAngle(state[kY]),
    //     state[kVx], state[kVy] };
}

arma::mat::fixed<BallModel::size, BallModel::size> BallModel::processNoise() {
    arma::mat noise = arma::eye(BallModel::size, BallModel::size) * processNoiseFactor;

    // noise(kX, kX) = processNoiseFactor * 100;
    // noise(kY, kY) = processNoiseFactor * 100;
    noise(kVx, kVx) = processNoiseFactor * 10;
    noise(kVy, kVy) = processNoiseFactor * 10;

    return noise;
}

}
}
}
