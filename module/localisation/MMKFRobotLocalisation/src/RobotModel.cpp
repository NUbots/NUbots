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

#include <nuclear>
#include <iostream>

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "message/localisation/FieldObject.h"
#include "utility/localisation/transform.h"
#include "message/input/Sensors.h"
#include "message/input/ServoID.h"

namespace module {
namespace localisation {
namespace robot {

    using message::input::Sensors;
    using message::input::ServoID;
    using utility::localisation::transform::SphericalRobotObservation;
    using utility::localisation::transform::WorldToRobotTransform;
    using utility::localisation::transform::RobotToWorldTransform;
    using utility::localisation::transform::ImuToWorldHeadingTransform;
    using utility::math::coordinates::cartesianToRadial;
    using utility::math::coordinates::cartesianToSpherical;

    arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(
        const arma::vec::fixed<RobotModel::size>& state, double /*deltaT*/, const Sensors& /*sensors*/) {
        arma::vec::fixed<RobotModel::size> new_state = state;

        return new_state;
    }


    /// Return the predicted observation of an object at the given position
    arma::vec RobotModel::predictedObservation(
        const arma::vec::fixed<RobotModel::size>& state,
        const arma::vec3& actual_position,
        const Sensors& sensors) {

        //Rewrite:
        double rmHeading = sensors.world.rotation().yaw() - state(robot::kImuOffset);
        arma::vec2 robotModelHeading = {std::cos(-rmHeading),std::sin(-rmHeading)};

        auto obs = SphericalRobotObservation(sensors.world.translation().rows(0,1) - state.rows(kX, kY),
                                             robotModelHeading,
                                             actual_position);
        return obs;
    }

    // Angle between goals - NOTE: CURRENTLY UNUSED
    arma::vec RobotModel::predictedObservation(
        const arma::vec::fixed<RobotModel::size>& state,
        const std::vector<arma::vec>& actual_positions,
        const Sensors& /*sensors*/) {

        //TODO: needs to incorporate new motion model position data
        arma::vec diff_1 = actual_positions[0].rows(0, 1) - state.rows(kX, kY);
        arma::vec diff_2 = actual_positions[1].rows(0, 1) - state.rows(kX, kY);
        arma::vec radial_1 = cartesianToRadial(diff_1);
        arma::vec radial_2 = cartesianToRadial(diff_2);

        auto angle_diff = utility::math::angle::difference(radial_1[1], radial_2[1]);

        return { std::abs(angle_diff) };
    }

    arma::vec RobotModel::observationDifference(const arma::vec& a,
                                                const arma::vec& b) {
        if (a.n_elem == 1) {
            return a - b;
        } if (a.n_elem == 2) {
            return a - b;
        } else {
            // Spherical coordinates
            arma::vec3 result = a - b;
            result(1) = /*utility::math::angle::normalizeAngle*/(result(1)) * cfg_.observationDifferenceBearingFactor;
            result(2) = /*utility::math::angle::normalizeAngle*/(result(2)) * cfg_.observationDifferenceElevationFactor;
            return result;
        }
    }

    arma::vec::fixed<RobotModel::size> RobotModel::limitState(
        const arma::vec::fixed<RobotModel::size>& state) {

        // TODO: Clip robot's state to the field.
        return state;
    }

    arma::mat::fixed<RobotModel::size, RobotModel::size> RobotModel::processNoise(){
        arma::mat noise = arma::eye(RobotModel::size, RobotModel::size);
        noise(kX, kX) *= cfg_.processNoisePositionFactor;
        noise(kY, kY) *= cfg_.processNoisePositionFactor;
        noise(kImuOffset, kImuOffset) *= cfg_.processNoiseHeadingFactor;
        // std::cout << "process noise = \n" << noise << std::endl;
        return noise;
    }


}
}
}
