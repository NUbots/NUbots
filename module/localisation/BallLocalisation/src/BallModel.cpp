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

#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/vision.h"

namespace module {
    namespace localisation {

        using utility::math::matrix::Rotation3D;
        using utility::math::matrix::Transform3D;
        using utility::math::matrix::Transform2D;
        using message::vision::Ball;
        using message::support::FieldDescription;
        using message::input::Sensors;
        using message::input::ServoID;

        arma::vec::fixed<BallModel::size> BallModel::timeUpdate(const arma::vec::fixed<size>& state, double /*deltaT*/) {
            return state;
        }

        arma::vec3 BallModel::predictedObservation(const arma::vec::fixed<size>& state
            , const FieldDescription& field
            , const Sensors& sensors
            , const MeasurementType::BALL&) const {

            // Get our transform to world coordinates
            const Transform3D& Htw = sensors.world;
            const Transform3D& Htc = sensors.forwardKinematics.find(ServoID::HEAD_PITCH)->second;
            Transform3D Hcw = Htc.i() * Htw;

            arma::vec3 rBWw = { state[PX], state[PY], field.ball_radius };

            return Hcw.transformPoint(rBWw);
        }

        arma::vec BallModel::observationDifference(const arma::vec& measurement
            , const arma::vec3& rBCc
            , const FieldDescription& field
            , const Sensors& /*sensors*/
            , const MeasurementType::BALL&) const {

            double len = arma::norm(rBCc);

            double expectedAngle = 2.0 * std::asin((field.ball_radius) / len);

            double actualAngle = std::acos(arma::dot(measurement, rBCc / len));

            return arma::vec({actualAngle - expectedAngle});
        }

        arma::vec::fixed<BallModel::size> BallModel::limitState(const arma::vec::fixed<size>& state) const {
            return state;
        }

        arma::mat::fixed<BallModel::size, BallModel::size> BallModel::processNoise() const {
            return arma::diagmat(processNoiseDiagonal);
        }

    }
}
