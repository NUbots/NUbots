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

#include "VirtualBall.h"

#include <armadillo>

#include "utility/input/ServoID.h"
#include "utility/math/coordinates.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/vision.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace support {

    using message::input::Image;
    using message::input::Sensors;
    using message::vision::Ball;
    using message::vision::Balls;

    using ServoID = utility::input::ServoID;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::math::vision::getFieldToCam;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::vision::screenToImage;

    VirtualBall::VirtualBall() : position(arma::fill::zeros), velocity(arma::fill::zeros), diameter(0.1), rd(rand()) {}

    VirtualBall::VirtualBall(arma::vec2 position, float diameter)
        : position({position[0], position[1], diameter * 0.5})
        , velocity(arma::fill::zeros)
        , diameter(diameter)
        , rd(rand()) {}

    // utility::math::matrix::Transform2D ballPose;
    arma::vec3 position;
    arma::vec3 velocity;

    // arma::vec2 position;
    float diameter;

    Balls VirtualBall::detect(const Image& image, Transform2D robotPose, const Sensors& sensors, arma::vec4 /*error*/) {

        Balls result;
        result.balls.reserve(1);

        Eigen::Affine3d Htc(sensors.forward_kinematics[utility::input::ServoID::HEAD_PITCH]);
        result.Hcw       = Htc.inverse() * sensors.Htw;
        result.timestamp = sensors.timestamp;  // TODO: Eventually allow this to be different to sensors.

        Transform3D Hcf = getFieldToCam(robotPose, convert(sensors.Hgc));
        Transform3D Hfc = Hcf.i();

        // Ball position in field
        arma::vec3 rBFf = position;

        // Camera position in field
        arma::vec3 rCFf = Hfc.translation();

        // Get our ball position in camera
        arma::vec3 rBCc = Hcf.rotation() * arma::vec3(rBFf - rCFf);
        if (rBCc[0] < 0.0) {
            return result;
        }

        double rBCcLength = arma::norm(rBCc);

        // The angular width of the cone we are drawing
        double angle = 2.0 * std::asin((diameter * 0.5) / arma::norm(rBCc));

        // Project the centre to the screen and work out the radius as if it was in the centre
        arma::ivec2 centre = screenToImage(projectCamSpaceToScreen(rBCc, image.lens), convert(image.dimensions));
        // TODO actually project this
        // double radius = 100 * std::tan(angle * 0.5);

        // Check our ball is on the screen at all and if so set the values
        if (centre[0] > 0 && centre[0] < int(image.dimensions[0]) && centre[1] > 0
            && centre[1] < int(image.dimensions[1])) {

            // Set our circle parameters for simulating the ball
            result.balls.at(0).cone.axis     = convert(arma::vec3(arma::normalise(rBCc))).cast<float>();
            result.balls.at(0).cone.gradient = std::tan(angle * 0.5);

            // Get our transform to world coordinates
            Transform3D Hwc = convert(Eigen::Matrix4d((Htc.inverse() * sensors.Htw).inverse()));

            arma::vec3 rBWw = Hwc.transformPoint(rBCc);
            // Attach the measurement to the object
            result.balls.at(0).measurements.push_back(Ball::Measurement());
            // TODO: This needs updating to actually provide rBCc
            result.balls.at(0).measurements.back().rBCc = convert(rBWw).cast<float>();

            // Measure points around the ball as a normal distribution
            arma::vec3 rEBc;
            if (rBCc[0] == 0.0 && rBCc[1] == 0.0) {
                if (rBCc[2] > 0.0) {
                    rEBc = {1, 0, 0};
                }
                else {
                    rEBc = {-1, 0, 0};
                }
            }
            else {
                // NOTE: this may not work correctly for view fields > 180 degrees
                rEBc = {M_SQRT1_2, 0, M_SQRT1_2};
            }
            // set rEBC to be a properly sized radius vector facing from the ball centre towards the (top or inner int
            // he case of extreme values) ball edge
            rEBc = rBCcLength * arma::normalise(rEBc - rEBc * arma::dot(rEBc, rBCc) / rBCcLength);
        }


        // If no measurements are in the Ball, then there it was not observed
        return result;
    }
}  // namespace support
}  // namespace module
