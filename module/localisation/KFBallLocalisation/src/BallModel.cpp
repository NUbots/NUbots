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


#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/localisation/transform.h"
#include "message/localisation/FieldObject.h"


// using message::localisation::FakeOdometry;
using utility::localisation::transform::SphericalRobotObservation;

namespace module {
namespace localisation {
namespace ball {

Eigen::Matrix<double, BallModel::size, 1> BallModel::ApplyVelocity(
    const Eigen::Matrix<double, BallModel::size, 1>& state, double deltaT) {
    auto result = state;

    // Apply ball velocity
    result(kX) += state(kVx) * deltaT;
    result(kY) += state(kVy) * deltaT;
    result(kVx) -= result(kVx) * cfg_.ballDragCoefficient * deltaT;
    result(kVy) -= result(kVy) * cfg_.ballDragCoefficient * deltaT;

    return result;
}

Eigen::Matrix<double, BallModel::size, 1> BallModel::timeUpdate(
    const Eigen::Matrix<double, BallModel::size, 1>& state, double deltaT) {

    return ApplyVelocity(state, deltaT);
}

// Eigen::Matrix<double, BallModel::size, 1> BallModel::timeUpdate(
//     const Eigen::Matrix<double, BallModel::size, 1>& state, double deltaT,
//     const FakeOdometry& odom) {

//     auto result = ApplyVelocity(state, deltaT);

//     // Apply robot odometry / robot position change
//     result.rows(kX, kY) -= odom.torso_displacement;

//     // Rotate ball_pos by -torso_rotation.
//     Eigen::Matrix2d rot = rotationMatrix(-odom.torso_rotation);
//     result.rows(kX, kY) = rot * result.rows(kX, kY);
//     result.rows(kVx, kVy) = rot * result.rows(kVx, kVy);

//     return result;
// }

/// Return the predicted observation of an object at the given position
Eigen::VectorXd BallModel::predictedObservation(
    const Eigen::Matrix<double, BallModel::size, 1>& state, double ballAngle) {

    Eigen::Vector3d ball_pos = Eigen::Vector3d(state(kX), state(kY), cfg_.ballHeight);
    auto obs = SphericalRobotObservation({0, 0}, 0, ball_pos);
    obs(1) -= ballAngle;

    Eigen::VectorXd obsVel = arma::join_cols(obs,state.rows(kVx,kVy));
    return obsVel;
}

Eigen::VectorXd BallModel::observationDifference(const arma::vec& a,
                                           const arma::vec& b){
    Eigen::VectorXd result = a - b;
    // result(1) = utility::math::angle::normalizeAngle(result(1));
    // result(2) = utility::math::angle::normalizeAngle(result(2));
    return result;
}

Eigen::Matrix<double, BallModel::size, 1> BallModel::limitState(
    const Eigen::Matrix<double, BallModel::size, 1>& state) {
    auto new_state = state;
    new_state.rows(kVx,kVy) = Eigen::Vector2d(0,0);
    return new_state;
}

Eigen::Matrix<double, BallModel::size, BallModel::size> BallModel::processNoise() {
    Eigen::Matrix<double, BallModel::size, BallModel::size> noise = Eigen::Matrix<double, BallModel::size, BallModel::size>::Identity();

    noise(kX, kX) *= cfg_.processNoisePositionFactor;
    noise(kY, kY) *= cfg_.processNoisePositionFactor;
    noise(kVx, kVx) *= cfg_.processNoiseVelocityFactor;
    noise(kVy, kVy) *= cfg_.processNoiseVelocityFactor;

    return noise;
}

}
}
}
