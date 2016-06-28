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

#include "VirtualGoalPost.h"

#include <armadillo>

#include "message/vision/VisionObjects.h"
#include "message/input/CameraParameters.h"
#include "message/input/Sensors.h"
#include "utility/math/coordinates.h"
#include "utility/math/geometry/Quad.h"
#include "utility/localisation/transform.h"
#include "utility/math/vision.h"
#include "message/support/FieldDescription.h"

namespace module {
namespace support {

    using message::vision::Goal;
    using utility::math::vision::cameraSpaceGoalProjection;
    using message::input::Sensors;
    using utility::math::matrix::Transform2D;
    using message::input::CameraParameters;
    using utility::math::geometry::Quad;
    using message::support::FieldDescription;

    arma::vec2 VirtualGoalPost::getCamRay(const arma::vec3& norm1, const arma::vec3& norm2, double focalLength, arma::uvec2 imSize) {
        //Solve the vector intersection between two planes to get the camera ray of the quad corner
        arma::vec3 result;
        const double zdiff = norm2[2]*norm1[1] - norm1[2] * norm2[1];
        const double ydiff = norm2[1]*norm1[2] - norm1[1] * norm2[2];
        if (std::abs(zdiff) > std::numeric_limits<double>::epsilon()) {
            result[0] = 1.0;
            result[2] = (norm1[0] * norm2[1] - norm1[1] * norm2[0]) / zdiff;
            result[1] = (-norm1[0] - norm1[2] * result[2]) / norm1[1];

        } else if (std::abs(ydiff) > std::numeric_limits<double>::epsilon()) {
            result[0] = 1.0;
            result[1] = (norm1[0] * norm2[2] - norm1[2] * norm2[0]) / ydiff;
            result[2] = (-norm1[0] - norm1[1] * result[1]) / norm1[2];

        } else {
            result[2] = 1.0;
            const double ndiff = norm1[0] * norm2[1] - norm1[1] * norm2[0];
            result[1] = (norm1[2] * norm2[0] - norm1[0] * norm2[2]) / ndiff;
            result[0] = (-norm1[2] - norm1[1] * result[1]) / norm1[0];
            if (result[0] < 0.0) {
                result *= -1.0;
            }
        }

        return arma::conv_to<arma::vec>::from(
                    utility::math::vision::screenToImage(
                        utility::math::vision::projectCamSpaceToScreen(result, focalLength),
                        imSize
                    )
                );
    }

    VirtualGoalPost::VirtualGoalPost(arma::vec3 position_, float height_, Goal::Side side_, Goal::Team team_){
        position = position_;
        height = height_;
        side = side_;
        team = team_;
    }

    Goal VirtualGoalPost::detect(const CameraParameters& camParams,
                Transform2D& robotPose,
                std::shared_ptr<const Sensors> sensors,
                arma::vec4& /*error*/,
                const FieldDescription& field){
        Goal result;


        //push the new measurement types
        //TODO: simulate not having values if off screen
        //std::cout << this->position;
        arma::mat::fixed<3,4> goalNormals = cameraSpaceGoalProjection(robotPose, this->position, field, *sensors);
        if (arma::any(arma::any(goalNormals > 0.0))) {
            result.measurements.push_back(
                std::make_pair<Goal::MeasurementType, arma::vec3>(
                            Goal::MeasurementType::LEFT_NORMAL,
                            goalNormals.col(0)
                        )
                );
            result.measurements.push_back(
                std::make_pair<Goal::MeasurementType, arma::vec3>(
                            Goal::MeasurementType::RIGHT_NORMAL,
                            goalNormals.col(1)
                        )
                );
            result.measurements.push_back(
                std::make_pair<Goal::MeasurementType, arma::vec3>(
                            Goal::MeasurementType::TOP_NORMAL,
                            goalNormals.col(2)
                        )
                );
            result.measurements.push_back(
                std::make_pair<Goal::MeasurementType, arma::vec3>(
                            Goal::MeasurementType::BASE_NORMAL,
                            goalNormals.col(3)
                        )
                );

            //build the predicted quad
            utility::math::geometry::Quad quad(
                    getCamRay(goalNormals.col(0), goalNormals.col(3), camParams.focalLengthPixels, camParams.imageSizePixels),
                    getCamRay(goalNormals.col(0), goalNormals.col(2), camParams.focalLengthPixels, camParams.imageSizePixels),
                    getCamRay(goalNormals.col(1), goalNormals.col(2), camParams.focalLengthPixels, camParams.imageSizePixels),
                    getCamRay(goalNormals.col(1), goalNormals.col(3), camParams.focalLengthPixels, camParams.imageSizePixels)
                );

            //goal base visibility check
            if (not (quad.getBottomRight()[1] > 0 && quad.getBottomRight()[1] < camParams.imageSizePixels[1]
                    && quad.getBottomLeft()[1] > 0 && quad.getBottomLeft()[1] < camParams.imageSizePixels[1]) ) {

                result.measurements.erase(result.measurements.begin()+3);
            }
            //goal top visibility check
            if (not (quad.getTopRight()[1] > 0 && quad.getTopRight()[1] < camParams.imageSizePixels[1]
                    && quad.getTopLeft()[1] > 0 && quad.getTopLeft()[1] < camParams.imageSizePixels[1]) ) {

                result.measurements.erase(result.measurements.begin()+2);
            }
            //goal sides visibility check
            if ( not (((quad.getBottomLeft()[0] > 0 && quad.getBottomLeft()[0] < camParams.imageSizePixels[0]
             && quad.getBottomRight()[0] > 0 && quad.getBottomRight()[0] < camParams.imageSizePixels[0])
             || (quad.getTopLeft()[0] > 0 && quad.getTopLeft()[0] < camParams.imageSizePixels[0]
             && quad.getTopRight()[0] > 0 && quad.getTopRight()[0] < camParams.imageSizePixels[0] ))
             && (quad.getBottomRight()[1] < 0 && quad.getTopRight()[1] > camParams.imageSizePixels[1]
             || quad.getBottomLeft()[1] < 0 && quad.getTopLeft()[1] > camParams.imageSizePixels[1]))

            ) {
                result.measurements.erase(result.measurements.begin()+1);
                result.measurements.erase(result.measurements.begin());
            }
            if (!result.measurements.empty()) {
                result.quad = quad;
            }
            /*std::cout << result.measurements.size() << std::endl
                      << quad.getTopLeft() << std::endl
                      << quad.getTopRight()  << std::endl
                      << quad.getBottomLeft() << std::endl
                      << quad.getBottomRight() << std::endl;*/
        }
        result.sensors = sensors;
        result.timestamp = sensors->timestamp; // TODO: Eventually allow this to be different to sensors.
        result.side = side;
        result.team = team;

        //If no measurements are in the goal, then it was not observed
        return result;
    }

}
}
