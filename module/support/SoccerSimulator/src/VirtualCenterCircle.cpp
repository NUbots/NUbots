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

#include "VirtualCenterCircle.h"

#include <armadillo>

#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "message/vision/CenterCircle.h"
#include "utility/input/ServoID.h"
#include "utility/math/coordinates.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/vision.h"


namespace module {
namespace support {
    using message::input::Image;
    using message::input::Sensors;
    using message::support::FieldDescription;
    using message::vision::CenterCircle;
    using message::vision::CenterCircles;
    using ServoID = utility::input::ServoID;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::math::vision::imageToScreen;

    VirtualCenterCircle::VirtualCenterCircle(arma::vec3 position_) {
        position = position_;
    }

    CenterCircles VirtualCenterCircle::detect(const Image& image,
                                              Transform2D& robotPose,
                                              const Sensors& sensors,
                                              arma::vec4& /*error*/,
                                              const FieldDescription& field) {
        CenterCircles result;
        result.centercircles.reserve(1);
        CenterCircle centercircle;
        // t = torso; c = camera; g = ground; f = foot;
        Eigen::Affine3d Htc(sensors.forward_kinematics[utility::input::ServoID::HEAD_PITCH]);
        result.Hcw       = Htc.inverse() * sensors.Htw;
        result.timestamp = sensors.timestamp;  // TODO: Eventually allow this to be different to sensors.

        Transform3D Hcf = getFieldToCam(robotPose, convert(sensors.Hgc));
        Transform3D Hfc = Hcf.i();

        // Center Circle (M) position in field (F)
        arma::vec3 rMFf = position;

        // Camera (C) position in field (F) (field is assumed to be equivalent to center circle)
        arma::vec3 rCFf = Hfc.translation();

        // Get our Center Circle (M) position in camera (C)
        arma::vec3 rMCc = Hcf.rotation() * arma::vec3(rCFf);
        if (rMCc[0] < 0.0) {
            return result;
        }

        double rMCc_norm = arma::norm(rMCc);

        // Project the centre to the screen
        arma::ivec2 centre = screenToImage(projectCamSpaceToScreen(rMCc, image.lens), convert(image.dimensions));

        // Check our ball is on the screen at all and if so set the values
        if (centre[0] > 0 && centre[0] < int(image.dimensions[0]) && centre[1] > 0
            && centre[1] < int(image.dimensions[1])) {

            // Get our transform to world coordinates
            Transform3D Hwc = convert(Eigen::Matrix4d((Htc.inverse() * sensors.Htw).inverse()));

            arma::vec3 rMWw = Hwc.transformPoint(rMCc);

            // rMCc_sphr and rMCc_cov
            float distance = 1.0;  // this needs to be actually calculated to replace this placeholder value
            auto rMCc_sphr = cartesianToSpherical(distance * rMCc_norm);
            Eigen::Vector3f VECTOR3_COVARIANCE({0.1, 0.001, 0.001});  // Taken from GoalDetector.yaml on June 2019

            Eigen::Vector3f covariance_amplifier({distance, 1, 1});
            Eigen::Matrix3f rMCc_cov = VECTOR3_COVARIANCE.cwiseProduct(covariance_amplifier).asDiagonal();

            if (std::isfinite(rMCc_sphr[0]) && std::isfinite(rMCc_sphr[1]) && std::isfinite(rMCc_sphr[2])) {
                centercircle.measurements.push_back(
                    CenterCircle::Measurement(CenterCircle::MeasurementType::CENTER, rMCc_sphr, rMCc_cov));
            }
        }
    }
}


}  // namespace support
}  // namespace module
