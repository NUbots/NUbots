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

#include "RobotModel.h"

#include <armadillo>
#include <nuclear>

#include "utility/math/angle.h"
#include "utility/math/matrix.h"
#include "utility/math/coordinates.h"
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"
#include "utility/localisation/transform.h"

using utility::localisation::transform::WorldToRobotTransform;
using messages::input::Sensors;
using messages::localisation::FakeOdometry;
using utility::math::matrix::rotationMatrix;
using utility::math::coordinates::cartesianToRadial;
using utility::math::coordinates::cartesianToSpherical;

namespace modules {
namespace localisation {
namespace robot {

// arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(
//     const arma::vec::fixed<RobotModel::size>& state, double deltaT) {
//         return state;
//     auto result = state;
//     return result;
// }

// arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(
//     const arma::vec::fixed<RobotModel::size>& state, double deltaT) {
    // auto result = state;

    // // Apply robot odometry / robot position change
    // arma::mat22 rot_heading = rotationMatrix(std::atan2(state[kHeadingY], state[kHeadingX]));
    // result.rows(kX, kY) += rot_heading * odom.torso_displacement;

    // // Rotate heading by torso_rotation.
    // arma::mat22 rot = rotationMatrix(odom.torso_rotation);
    // result.rows(kHeadingX, kHeadingY) = rot * result.rows(kHeadingX, kHeadingY);

    // return result;
// }

arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(
    const arma::vec::fixed<RobotModel::size>& state, double) {
    return state;
    // auto result = state;
    // arma::mat44 odom = sensors.odometry;
    // arma::vec3 updated_heading3 = odom.submat(0,0,2,2).t() * arma::vec3({state[kHeadingX], state[kHeadingY],0});
    // arma::vec2 updated_heading = arma::normalise(updated_heading3.rows(0,1));
    // arma::vec4 updated_position = arma::vec4({state[kX], state[kY], 0, 1}) + odom * arma::vec4({0, 0, 0, 1});

    // if(arma::norm(updated_heading) > 0){
    //     return {updated_position[0], updated_position[1], updated_heading[0], updated_heading[1]};
    // } else {
    //     return {updated_position[0], updated_position[1], state[kHeadingX], state[kHeadingY]};
    // }
}


/// Return the predicted observation of an object at the given position
    arma::vec RobotModel::predictedObservation(
        const arma::vec::fixed<RobotModel::size>& state, const arma::vec3& actual_position) {
    // arma::mat worldToRobot = getWorldToRobotTransform(state);
    // // arma::mat robotToWorld = getRobotToWorldTransform(state);
    // // arma::mat identity = worldToRobot * robotToWorld;
    // // NUClear::log("worldToRobot\n",worldToRobot, "\nrobotToWorld\n",robotToWorld, "\nidentity\n",identity);
    // arma::vec objectPosition = arma::vec({actual_position[0], actual_position[1], 1});
    // arma::vec expectedObservation = worldToRobot * objectPosition;
    // return cartesianToRadial(expectedObservation.rows(0, 1));

    auto actual_pos_robot_2d = WorldToRobotTransform(state.rows(kX, kY),
                                                     state(kHeading),
                                                     actual_position.rows(0, 1));
    auto actual_pos_robot_3d = arma::vec3({actual_pos_robot_2d(0),
                                           actual_pos_robot_2d(1),
                                           actual_position(2)});
    return cartesianToSpherical(actual_pos_robot_3d);

    // NUClear::log("worldToRobot =\n", worldToRobot);
    // NUClear::log("objectPosition =\n", objectPosition);
    // NUClear::log("predictedObservation =\n", expectedObservation);

    // // // Radial coordinates
    // arma::vec2 diff = actual_position - state.rows(kX, kY);
    // arma::vec2 radial = utility::math::coordinates::cartesianToRadial(diff);
    // // radial(1) = utility::math::angle::normalizeAngle(radial[1] - state[kHeading]);
    // // return radial;

    // auto heading_angle = std::atan2(state[kHeadingY], state[kHeadingX]);

    // // Distance and unit vector heading
    // heading_angle = utility::math::angle::normalizeAngle(radial[1] - heading_angle);

    // auto heading_x = std::cos(heading_angle);
    // auto heading_y = std::sin(heading_angle);

    // // arma::vec2 heading = arma::normalise(diff);
    // return {radial[0], heading_x, heading_y};
}

// Angle between goals
arma::vec RobotModel::predictedObservation(
    const arma::vec::fixed<RobotModel::size>& state,
    const std::vector<arma::vec>& actual_positions) {

    // // Radial coordinates
    arma::vec diff_1 = actual_positions[0] - state.rows(kX, kY);
    arma::vec diff_2 = actual_positions[1] - state.rows(kX, kY);
    arma::vec radial_1 = cartesianToRadial(diff_1);
    arma::vec radial_2 = cartesianToRadial(diff_2);

    auto angle_diff = utility::math::angle::difference(radial_1[1], radial_2[1]);


    return { std::abs(angle_diff) };
}


arma::vec RobotModel::observationDifference(const arma::vec& a,
                                            const arma::vec& b) {
    if (a.n_elem == 1) {
        return a - b;
    } else {
        // Spherical coordinates
        arma::vec3 result = a - b;
        // result(1) = utility::math::angle::normalizeAngle(result[1]);
        result(1) = utility::math::angle::difference(a(1), b(1));
        result(2) = utility::math::angle::difference(a(2), b(2));
        return result;
    }

    // // Distance and unit vector heading
    // return a - b;
    // arma::vec2 result = a - b;
    // arma::vec2 heading_diff = {result[1], result[2]};
    // arma::vec2 heading = arma::normalise(heading_diff);
    // return {result[0], heading[0], heading[1]};
}


arma::vec::fixed<RobotModel::size> RobotModel::limitState(
    const arma::vec::fixed<RobotModel::size>& state) {

    return state;

    // auto result = state;
    // // // // How to get clipping values from config system?
    // // // result[kX] = std::max(std::min(result[kX], 4.5 + 0.7) , -4.5 -0.7);
    // // // result[kY] = std::max(std::min(result[kY], 3 + 0.7) , -3 -0.7);

    // // // Radial coordinates
    // // result[kHeading] = utility::math::angle::normalizeAngle(result[kHeading]);
    // return result;


    // // Unit vector orientation
    // arma::vec2 heading = { state[kHeadingX], state[kHeadingY] };
    // arma::vec2 unit = arma::normalise(heading);
    // return arma::vec({ state[kX], state[kY], state[kHeadingX], state[kHeadingY] });
}

arma::mat::fixed<RobotModel::size, RobotModel::size> RobotModel::processNoise() {
    return arma::eye(RobotModel::size, RobotModel::size) * processNoiseFactor;
}

// arma::mat33 RobotModel::getRobotToWorldTransform(const arma::vec::fixed<RobotModel::size>& state){
//     arma::vec2 normed_heading = arma::normalise(state.rows(kHeadingX,kHeadingY));
//     arma::mat33 T;

//     T << normed_heading[0] << -normed_heading[1] << state[kX] << arma::endr
//       << normed_heading[1] <<  normed_heading[0] << state[kY] << arma::endr
//       <<                 0 <<                  0 <<         1;

//     return T;
// }

// arma::mat33 RobotModel::getWorldToRobotTransform(const arma::vec::fixed<RobotModel::size>& state){
//     arma::vec2 normed_heading = arma::normalise(state.rows(kHeadingX,kHeadingY));
//     arma::mat33 Tinverse;
//     Tinverse << normed_heading[0] <<  normed_heading[1] <<         0 << arma::endr
//              <<-normed_heading[1] <<  normed_heading[0] <<         0 << arma::endr
//              <<                 0 <<                  0 <<         1;

//     Tinverse.submat(0,2,1,2) = -Tinverse.submat(0,0,1,1) * arma::vec2({state[kX], state[kY]});
//     return Tinverse;
// }

}
}
}
