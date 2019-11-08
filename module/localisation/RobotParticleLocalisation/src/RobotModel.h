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

#ifndef MODULES_LOCALISATION_ROBOTMODEL_H
#define MODULES_LOCALISATION_ROBOTMODEL_H

#include <armadillo>

#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "message/vision/Goal.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"


namespace module {
namespace localisation {

    class RobotModel {
    public:
        static constexpr size_t size = 3;

        enum Components : int {
            // Field center in world space
            kX = 0,
            kY = 1,
            // Angle is the angle from the robot world forward direction to the field forward direction
            kAngle = 2
        };


        RobotModel() {}

        arma::vec::fixed<RobotModel::size> timeUpdate(const arma::vec::fixed<RobotModel::size>& state, double deltaT);

        arma::vec predictedObservation(const arma::vec::fixed<RobotModel::size>& state,
                                       const arma::vec& actual_position,
                                       const utility::math::matrix::Transform3D& Hcw,
                                       const message::vision::Goal::MeasurementType& type,
                                       const message::support::FieldDescription& fd);

        arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

        arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);

        arma::mat::fixed<size, size> processNoise();

        arma::vec3 processNoiseDiagonal;

        // number and range of reset particles
        int n_rogues          = 0;
        arma::vec3 resetRange = {0, 0, 0};

        // Getters
        int getRogueCount() const {
            return n_rogues;
        }
        arma::vec getRogueRange() const {
            return resetRange;
        }

        arma::vec3 getCylindricalPostCamSpaceNormal(const message::vision::Goal::MeasurementType& type,
                                                    const arma::vec3& post_centre,
                                                    const utility::math::matrix::Transform3D& Hcf,
                                                    const message::support::FieldDescription& fd);
        arma::vec3 getSquarePostCamSpaceNormal(const message::vision::Goal::MeasurementType& type,
                                               const arma::vec3& post_centre,
                                               const utility::math::matrix::Transform3D& Hcf,
                                               const message::support::FieldDescription& fd);
    };
}  // namespace localisation
}  // namespace module
#endif
