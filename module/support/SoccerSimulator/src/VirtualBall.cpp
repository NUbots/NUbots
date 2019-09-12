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

#include "utility/input/ServoID.h"
#include "utility/math/coordinates.h"
#include "utility/math/vision.h"

namespace module {
namespace support {

    using message::input::Image;
    using message::input::Sensors;
    using message::vision::Ball;
    using message::vision::Balls;

    using utility::input::ServoID;
    using utility::math::vision::getFieldToCam;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::vision::screenToImage;

    VirtualBall::VirtualBall()
        : position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()), diameter(0.1), rd(rand()) {}

    VirtualBall::VirtualBall(const Eigen::Vector2d& position, float diameter)
        : position(position.x(), position.y(), diameter * 0.5)
        , velocity(arma::fill::zeros)
        , diameter(diameter)
        , rd(rand()) {}

    // utility::math::matrix::Transform2D ballPose;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;

    // arma::vec2 position;
    float diameter;

    Balls VirtualBall::detect(const Image& image,
                              const Eigen::Affine2d& robotPose,
                              const Sensors& sensors,
                              const Eigen::Vector4d& /*error*/) {

        Balls result;
        result.balls.reserve(1);

        Eigen::Affine3d Htc(sensors.forward_kinematics[utility::input::ServoID::HEAD_PITCH]);
        result.Hcw       = Htc.inverse() * sensors.Htw;
        result.timestamp = sensors.timestamp;  // TODO: Eventually allow this to be different to sensors.

        Eigen::Affine3d Hcf = getFieldToCam(robotPose, sensors.Hgc);
        Eigen::Affine3d Hfc = Hcf.inverse();

        // Ball position in field
        Eigen::Vector3d rBFf = position;

        // Camera position in field
        Eigen::Vector3d rCFf = Hfc.translation();

        // Get our ball position in camera
        Eigen::Vector3d rBCc = Hcf.rotation() * (rBFf - rCFf);
        if (rBCc.x() < 0.0) {
            return result;
        }

        double rBCcLength = rBCc.norm();

        // The angular width of the cone we are drawing
        double angle = 2.0 * std::asin((diameter * 0.5) / rBCc.norm());

        // Project the centre to the screen and work out the radius as if it was in the centre
        auto centre = screenToImage(projectCamSpaceToScreen(rBCc, image.lens), image.dimensions);
        // TODO actually project this
        // double radius = 100 * std::tan(angle * 0.5);

        // Check our ball is on the screen at all and if so set the values
        if (centre.x() > 0 && centre.x() < int(image.dimensions.x()) && centre.y() > 0
            && centre.y() < int(image.dimensions.y())) {

            // Set our circle parameters for simulating the ball
            result.balls.at(0).cone.axis     = rBCc.normalized().cast<float>();
            result.balls.at(0).cone.gradient = std::tan(angle * 0.5);

            // Get our transform to world coordinates
            Eigen::Affine3d Hwc((Htc.inverse() * sensors.Htw).inverse());

            Eigen::Vector3d rBWw = (Hwc * Eigen::Vector4d(rBCc.x(), rBCc.y(), rBCc.z(), 1.0))).head<3>();
            // Attach the measurement to the object
            result.balls.at(0).measurements.push_back(Ball::Measurement());
            // TODO: This needs updating to actually provide rBCc
            result.balls.at(0).measurements.back().rBCc = rBWw.cast<float>();

            // Measure points around the ball as a normal distribution
            Eigen::Vector3d rEBc;
            if (rBCc.x() == 0.0 && rBCc.y() == 0.0) {
                if (rBCc.z() > 0.0) {
                    rEBc = Eigen::Vector3d::UnitX();
                }
                else {
                    rEBc = -Eigen::Vector3d::UnitX();
                }
            }
            else {
                // NOTE: this may not work correctly for view fields > 180 degrees
                rEBc = Eigen::Vector3d(M_SQRT1_2, 0.0, M_SQRT1_2);
            }
            // set rEBC to be a properly sized radius vector facing from the ball centre towards the (top or inner int
            // he case of extreme values) ball edge
            rEBc = rBCcLength * (rEBc - rEBc * rEBc.dot(rBCc) / rBCcLength).normalized();
        }

        // If no measurements are in the Ball, then there it was not observed
        return result;
    }
}  // namespace support
}  // namespace module
