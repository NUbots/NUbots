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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "VirtualGoalPost.h"

#include <armadillo>

#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "message/vision/Goal.h"

#include "utility/input/ServoID.h"
#include "utility/math/coordinates.h"
#include "utility/math/geometry/Quad.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/vision.h"

namespace module {
namespace support {

    using message::input::Image;
    using message::input::Sensors;
    using message::support::FieldDescription;
    using message::vision::Goal;
    using message::vision::Goals;
    using ServoID = utility::input::ServoID;

    using utility::math::geometry::Quad;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::math::vision::cameraSpaceGoalProjection;

    arma::vec2 VirtualGoalPost::getCamRay(const arma::vec3& norm1,
                                          const arma::vec3& norm2,
                                          const Image::Lens& lens,
                                          arma::uvec2 dimensions) {
        // Solve the vector intersection between two planes to get the camera ray of the quad corner
        arma::vec3 result;
        const double zdiff = norm2[2] * norm1[1] - norm1[2] * norm2[1];
        const double ydiff = norm2[1] * norm1[2] - norm1[1] * norm2[2];
        if (std::abs(zdiff) > std::numeric_limits<double>::epsilon()) {
            result[0] = 1.0;
            result[2] = (norm1[0] * norm2[1] - norm1[1] * norm2[0]) / zdiff;
            result[1] = (-norm1[0] - norm1[2] * result[2]) / norm1[1];
        }
        else if (std::abs(ydiff) > std::numeric_limits<double>::epsilon()) {
            result[0] = 1.0;
            result[1] = (norm1[0] * norm2[2] - norm1[2] * norm2[0]) / ydiff;
            result[2] = (-norm1[0] - norm1[1] * result[1]) / norm1[2];
        }
        else {
            result[2]          = 1.0;
            const double ndiff = norm1[0] * norm2[1] - norm1[1] * norm2[0];
            result[1]          = (norm1[2] * norm2[0] - norm1[0] * norm2[2]) / ndiff;
            result[0]          = (-norm1[2] - norm1[1] * result[1]) / norm1[0];
            if (result[0] < 0.0) {
                result *= -1.0;
            }
        }

        return arma::conv_to<arma::vec>::from(utility::math::vision::screenToImage(
            utility::math::vision::projectCamSpaceToScreen(result, lens), dimensions));
    }

    VirtualGoalPost::VirtualGoalPost(arma::vec3 position_, float height_, Goal::Side side_, Goal::Team team_) {
        position = position_;
        height   = height_;
        side     = side_;
        team     = team_;
    }

    Goals VirtualGoalPost::detect(const Image& image,
                                  Transform2D& robotPose,
                                  const Sensors& sensors,
                                  arma::vec4& /*error*/,
                                  const FieldDescription& field) {
        Goals result;
        result.goals.reserve(1);

        // t = torso; c = camera; g = ground; f = foot;
        Eigen::Affine3d Htc(sensors.forward_kinematics[utility::input::ServoID::HEAD_PITCH]);
        result.Hcw              = Htc.inverse() * sensors.Htw;
        result.timestamp        = sensors.timestamp;  // TODO: Eventually allow this to be different to sensors.
        result.goals.at(0).side = side;
        result.goals.at(0).team = team;

        // get the torso to foot transform
        Transform3D Hgt  = convert(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);
        Transform3D Hgt2 = convert(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]);

        if (Hgt2(3, 2) < Hgt(3, 2)) {
            Hgt = Hgt2;
        }
        // remove translation components from the transform
        Hgt.submat(0, 3, 1, 3) *= 0.0;
        Hgt.rotation() = Rotation3D::createRotationZ(-Rotation3D(Hgt.rotation()).yaw()) * Hgt.rotation();
        // create the camera to ground transform
        Transform3D Hgc = Hgt * convert(Htc.matrix());

        // push the new measurement types

        arma::mat::fixed<3, 4> goalNormals = cameraSpaceGoalProjection(robotPose, this->position, field, Hgc);
        if (arma::any(arma::any(goalNormals > 0.0))) {
            result.goals.at(0).measurements.push_back(Goal::Measurement(
                Goal::MeasurementType::LEFT_NORMAL, convert(arma::vec3(goalNormals.col(0))).cast<float>()));
            result.goals.at(0).measurements.push_back(Goal::Measurement(
                Goal::MeasurementType::RIGHT_NORMAL, convert(arma::vec3(goalNormals.col(1))).cast<float>()));
            result.goals.at(0).measurements.push_back(Goal::Measurement(
                Goal::MeasurementType::TOP_NORMAL, convert(arma::vec3(goalNormals.col(2))).cast<float>()));
            result.goals.at(0).measurements.push_back(Goal::Measurement(
                Goal::MeasurementType::BASE_NORMAL, convert(arma::vec3(goalNormals.col(3))).cast<float>()));

            // build the predicted quad
            utility::math::geometry::Quad quad(
                getCamRay(goalNormals.col(0), goalNormals.col(3), image.lens, convert(image.dimensions)),
                getCamRay(goalNormals.col(0), goalNormals.col(2), image.lens, convert(image.dimensions)),
                getCamRay(goalNormals.col(1), goalNormals.col(2), image.lens, convert(image.dimensions)),
                getCamRay(goalNormals.col(1), goalNormals.col(3), image.lens, convert(image.dimensions)));


            // goal base visibility check
            if (not(quad.getBottomRight()[1] > 0 && quad.getBottomRight()[1] < image.dimensions[1]
                    && quad.getBottomLeft()[1] > 0 && quad.getBottomLeft()[1] < image.dimensions[1]
                    && quad.getBottomRight()[0] > 0 && quad.getBottomRight()[0] < image.dimensions[0]
                    && quad.getBottomLeft()[0] > 0 && quad.getBottomLeft()[0] < image.dimensions[0])) {

                result.goals.at(0).measurements.erase(result.goals.at(0).measurements.begin() + 3);
            }
            // goal top visibility check
            if (not(quad.getTopRight()[1] > 0 && quad.getTopRight()[1] < image.dimensions[1] && quad.getTopLeft()[1] > 0
                    && quad.getTopLeft()[1] < image.dimensions[1] && quad.getTopRight()[0] > 0
                    && quad.getTopRight()[0] < image.dimensions[0] && quad.getTopLeft()[0] > 0
                    && quad.getTopLeft()[0] < image.dimensions[0])) {

                result.goals.at(0).measurements.erase(result.goals.at(0).measurements.begin() + 2);
            }
            // goal sides visibility check
            if (not((
                        // One of the top or the bottom are in the screen coordinates of x
                        (quad.getBottomLeft()[0] > 0 && quad.getBottomLeft()[0] < image.dimensions[0]
                         && quad.getBottomRight()[0] > 0 && quad.getBottomRight()[0] < image.dimensions[0])
                        || (quad.getTopLeft()[0] > 0 && quad.getTopLeft()[0] < image.dimensions[0]
                            && quad.getTopRight()[0] > 0 && quad.getTopRight()[0] < image.dimensions[0]))
                    && (
                           // Check that the bottom is below the top of the screen and the top is below the bottom of
                           // the screen
                           (quad.getBottomRight()[1] < image.dimensions[1] && quad.getTopRight()[1] > 0)
                           || (quad.getBottomLeft()[1] < image.dimensions[1] && quad.getTopLeft()[1] > 0)))) {
                result.goals.at(0).measurements.erase(result.goals.at(0).measurements.begin() + 1);
                result.goals.at(0).measurements.erase(result.goals.at(0).measurements.begin());
            }
            // if (!result.goals.at(0).measurements.empty()) {
            //     result.goals.at(0).post.top    = (convert(quad.getTopLeft()) + convert(quad.getTopRight()) * 0.5);
            //     result.goals.at(0).post.bottom = (convert(quad.getBottomLeft()) + convert(quad.getBottomRight()) *
            //     0.5);
            // }
        }

        // If no measurements are in the goal, then it was not observed
        return result;
    }
}  // namespace support
}  // namespace module
