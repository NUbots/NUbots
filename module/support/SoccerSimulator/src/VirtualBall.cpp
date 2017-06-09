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

#include "VirtualBall.h"

#include <Eigen/Core>

#include "utility/input/ServoID.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/coordinates.h"
#include "utility/math/vision.h"

namespace module {
namespace support {

    using message::input::CameraParameters;
    using message::input::Sensors;
    using message::vision::Ball;
    using ServoID = utility::input::ServoID;

    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::math::matrix::Rotation3D;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::vision::screenToImage;
    using utility::math::vision::getFieldToCam;

    VirtualBall::VirtualBall()
    : position(arma::fill::zeros)
    , velocity(arma::fill::zeros)
    , diameter(0.1)
    , rd(rand()) {
    }

    VirtualBall::VirtualBall(Eigen::Vector2d position, float diameter)
    : position({position[0], position[1], diameter * 0.5})
    , velocity(arma::fill::zeros)
    , diameter(diameter)
    , rd(rand()) {
    }

    // utility::math::matrix::Transform2D ballPose;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;

    // Eigen::Vector2d position;
    float diameter;

    Ball VirtualBall::detect(const CameraParameters& cam, Transform2D robotPose, const Sensors& sensors, Eigen::Vector4d /*error*/){

        Ball result;

        Transform3D Hcf = getFieldToCam(robotPose, sensors.camToGround);
        Transform3D Hfc = Hcf.i();

        // Ball position in field
        Eigen::Vector3d rBFf = position;

        // Camera position in field
        Eigen::Vector3d rCFf = Hfc.translation();

        // Get our ball position in camera
        Eigen::Vector3d rBCc = Hcf.rotation() * Eigen::Vector3d(rBFf - rCFf);
        if (rBCc[0] < 0.0) {
            result.edgePoints.clear();
            return result;
        }


        double rBCcLength = rBCc.norm();

        // The angular width of the cone we are drawing
        double angle = 2.0 * std::asin((diameter * 0.5) / rBCc.norm());

        // Project the centre to the screen and work out the radius as if it was in the centre
        Eigen::Vector2i centre = screenToImage(projectCamSpaceToScreen(rBCc, cam.focalLengthPixels), cam.imageSizePixels);
        double radius = cam.focalLengthPixels * std::tan(angle * 0.5);

        // Check our ball is on the screen at all and if so set the values
        if (  centre[0] > 0
           && centre[0] < int(cam.imageSizePixels[0])
           && centre[1] > 0
           && centre[1] < int(cam.imageSizePixels[1])) {

            // Set our circle parameters for simulating the ball
            result.circle.centre = arma::conv_to<arma::vec>::from(centre);
            result.circle.radius = radius;

            // Get our transform to world coordinates
            const Transform3D& Htw = sensors.world;
            const Transform3D& Htc = sensors.forwardKinematics.at(ServoID::HEAD_PITCH);
            Transform3D Hcw = Htc.i() * Htw;
            Transform3D Hwc = Hcw.i();

            result.position = Hwc.transformPoint(rBCc);

            // Measure points around the ball as a normal distribution
            Eigen::Vector3d rEBc;
            if (rBCc[0] == 0.0 && rBCc[1] == 0.0 ) {
                if (rBCc[2] > 0.0) {
                    rEBc = { 1, 0, 0};
                } else {
                    rEBc = { -1, 0, 0};
                }
            }
            else {
                //NOTE: this may not work correctly for view fields > 180 degrees
                rEBc = { M_SQRT1_2, 0, M_SQRT1_2 };
            }
            //set rEBC to be a properly sized radius vector facing from the ball centre towards the (top or inner int he case of extreme values) ball edge
            rEBc = rBCcLength * (rEBc - rEBc * rEBc.dot(rBCc) / rBCcLength).normalize();


            for (int i = 0; i < 50; ++i) {
                //
                double radialJitter = radialDistribution(rd);
                double angleOffset = angularDistribution(rd);

                // Get a random number for which direciton the measurement is
                Eigen::Vector3d rEBc = rEBc * std::tan(angle + radialJitter / 2.0);

                // Make a rotation matrix to rotate our vector to our target
                // Eigen lpNorm<p> is templated on p ... so p must be known at compile time.
                // Introducing the fucked-up hack!!
                //result.edgePoints.push_back(Rotation3D(arma::normalise(rBCc, angle + angleOffset) * rEBc).normalize());
                result.edgePoints.push_back(Rotation3D(std::pow(rBCc.pow(angle + angleOffset).sum(), 1 / (angle + angleOffset)) * rEBc).normalize());
            }
        }

        result.visObject.sensors = const_cast<Sensors*>(&sensors)->shared_from_this();
        result.visObject.timestamp = sensors.timestamp; // TODO: Eventually allow this to be different to sensors.


        //If no measurements are in the Ball, then there it was not observed
        return result;
    }

}
}
