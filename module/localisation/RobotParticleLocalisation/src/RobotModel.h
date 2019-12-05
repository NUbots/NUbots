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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <armadillo>

#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "message/vision/Goal.h"


namespace module {
namespace localisation {

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


        RobotModel() {}

        StateVec time(const StateVec& state, double deltaT);

        Eigen::VectorXd predictedObservation(const StateVec& state,
                                             const Eigen::VectorXd& actual_position,
                                             const Eigen::Affine3d& Hcw,
                                             const message::vision::Goal::MeasurementType& type,
                                             const message::support::FieldDescription& fd);

        StateVec limit(const StateVec& state);

        StateMat noise(const Scalar& deltaT);

        template <typename... Args>
        Eigen::Matrix<Scalar, 1, 1> predict(const StateVec& state, const Args&... params) {

            // Our prediction is the first state
            return Eigen::Matrix<Scalar, 1, 1>(state[kX]);
        }

        template <typename T, typename U>
        static auto difference(const T& a, const U& b) {
            return a - b;
        }

        Eigen::Vector3d processNoiseDiagonal;

        // number and range of reset particles
        int n_rogues               = 0;
        Eigen::Vector3d resetRange = {0, 0, 0};

        // Getters
        int getRogueCount() const {
            return n_rogues;
        }
        Eigen::VectorXd getRogueRange() const {
            return resetRange;
        }

        Eigen::Vector3d getCylindricalPostCamSpaceNormal(const message::vision::Goal::MeasurementType& type,
                                                         const Eigen::Vector3d& post_centre,
                                                         const Eigen::Affine3d& Hcf,
                                                         const message::support::FieldDescription& fd);
        Eigen::Vector3d getSquarePostCamSpaceNormal(const message::vision::Goal::MeasurementType& type,
                                                    const Eigen::Vector3d& post_centre,
                                                    const Eigen::Affine3d& Hcf,
                                                    const message::support::FieldDescription& fd);
    };
}  // namespace localisation
}  // namespace module
#endif
