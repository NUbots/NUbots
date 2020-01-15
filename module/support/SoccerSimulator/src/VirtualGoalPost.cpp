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

#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "message/vision/Goal.h"
#include "utility/input/ServoID.h"
#include "utility/math/coordinates.h"
#include "utility/math/geometry/Quad.h"

namespace module {
namespace support {

    using message::input::Image;
    using message::input::Sensors;
    using message::support::FieldDescription;
    using message::vision::Goal;
    using message::vision::Goals;
    using utility::input::ServoID;

    using utility::math::geometry::Quad;

    template <typename Scalar>
    Eigen::Matrix<Scalar, 2, 1> VirtualGoalPost::getCamRay(const Eigen::Matrix<Scalar, 3, 1>& norm1,
                                                           const Eigen::Matrix<Scalar, 3, 1>& norm2,
                                                           const Image::Lens& lens,
                                                           const Eigen::Matrix<unsigned int, 2, 1>& dimensions) {
        // Solve the vector intersection between two planes to get the camera ray of the quad corner
        Eigen::Matrix<Scalar, 3, 1> result;
        const Scalar zdiff = norm2.z() * norm1.y() - norm1.z() * norm2.y();
        const Scalar ydiff = norm2.y() * norm1.z() - norm1.y() * norm2.z();
        if (std::abs(zdiff) > std::numeric_limits<Scalar>::epsilon()) {
            result.x() = 1.0;
            result.z() = (norm1.x() * norm2.y() - norm1.y() * norm2.x()) / zdiff;
            result.y() = (-norm1.x() - norm1.z() * result.z()) / norm1.y();
        }
        else if (std::abs(ydiff) > std::numeric_limits<Scalar>::epsilon()) {
            result.x() = 1.0;
            result.y() = (norm1.x() * norm2.z() - norm1.z() * norm2.x()) / ydiff;
            result.z() = (-norm1.x() - norm1.y() * result.y()) / norm1.z();
        }
        else {
            result.z()         = 1.0;
            const Scalar ndiff = norm1.x() * norm2.y() - norm1.y() * norm2.x();
            result.y()         = (norm1.z() * norm2.x() - norm1.x() * norm2.z()) / ndiff;
            result.x()         = (-norm1.z() - norm1.y() * result.y()) / norm1.x();
            if (result.x() < 0.0) {
                result *= -1.0;
            }
        }

        return screenToImage<Scalar>(projectCamSpaceToScreen<Scalar>(result, lens), dimensions);
    }

    Goals VirtualGoalPost::detect(const Image& image,
                                  const Eigen::Affine2d& robotPose,
                                  const Sensors& sensors,
                                  const Eigen::Vector4d& /*error*/,
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
        Eigen::Affine3d Hgt(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);
        Eigen::Affine3d Hgt2(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]);

        if (Hgt2(3, 2) < Hgt(3, 2)) {
            Hgt = Hgt2;
        }
        // remove translation components from the transform
        Hgt.translation() = Eigen::Vector3d::Zero();
        Hgt.linear() =
            Eigen::AngleAxisd(-Hgt.linear().matrix().eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ()) * Hgt.linear();
        // create the camera to ground transform
        Eigen::Affine3d Hgc = Hgt * Htc;

        // push the new measurement types

        Eigen::Matrix<float, 3, 4> goalNormals =
            cameraSpaceGoalProjection(robotPose, this->position, field, Hgc).cast<float>();
        if ((goalNormals.array() > 0.0).any()) {
            result.goals.at(0).measurements.push_back(
                Goal::Measurement(Goal::MeasurementType::LEFT_NORMAL, goalNormals.block<3, 1>(0, 0)));
            result.goals.at(0).measurements.push_back(
                Goal::Measurement(Goal::MeasurementType::RIGHT_NORMAL, goalNormals.block<3, 1>(0, 1)));
            result.goals.at(0).measurements.push_back(
                Goal::Measurement(Goal::MeasurementType::TOP_NORMAL, goalNormals.block<3, 1>(0, 2)));
            result.goals.at(0).measurements.push_back(
                Goal::Measurement(Goal::MeasurementType::BASE_NORMAL, goalNormals.block<3, 1>(0, 3)));

            // build the predicted quad
            utility::math::geometry::Quad<Eigen::Vector2f> quad(
                getCamRay<float>(
                    goalNormals.block<3, 1>(0, 0), goalNormals.block<3, 1>(0, 3), image.lens, image.dimensions)
                    .cast<float>(),
                getCamRay<float>(
                    goalNormals.block<3, 1>(0, 0), goalNormals.block<3, 1>(0, 2), image.lens, image.dimensions)
                    .cast<float>(),
                getCamRay<float>(
                    goalNormals.block<3, 1>(0, 1), goalNormals.block<3, 1>(0, 2), image.lens, image.dimensions)
                    .cast<float>(),
                getCamRay<float>(
                    goalNormals.block<3, 1>(0, 1), goalNormals.block<3, 1>(0, 3), image.lens, image.dimensions)
                    .cast<float>());


            // goal base visibility check
            if (!(quad.getBottomRight().y() > 0 && quad.getBottomRight().y() < image.dimensions.y()
                  && quad.getBottomLeft().y() > 0 && quad.getBottomLeft().y() < image.dimensions.y()
                  && quad.getBottomRight().x() > 0 && quad.getBottomRight().x() < image.dimensions.x()
                  && quad.getBottomLeft().x() > 0 && quad.getBottomLeft().x() < image.dimensions.x())) {

                result.goals.at(0).measurements.erase(result.goals.at(0).measurements.begin() + 3);
            }
            // goal top visibility check
            if (!(quad.getTopRight().y() > 0 && quad.getTopRight().y() < image.dimensions.y()
                  && quad.getTopLeft().y() > 0 && quad.getTopLeft().y() < image.dimensions.y()
                  && quad.getTopRight().x() > 0 && quad.getTopRight().x() < image.dimensions.x()
                  && quad.getTopLeft().x() > 0 && quad.getTopLeft().x() < image.dimensions.x())) {

                result.goals.at(0).measurements.erase(result.goals.at(0).measurements.begin() + 2);
            }
            // goal sides visibility check
            if (!((
                      // One of the top or the bottom are in the screen coordinates of x
                      (quad.getBottomLeft().x() > 0 && quad.getBottomLeft().x() < image.dimensions.x()
                       && quad.getBottomRight().x() > 0 && quad.getBottomRight().x() < image.dimensions.x())
                      || (quad.getTopLeft().x() > 0 && quad.getTopLeft().x() < image.dimensions.x()
                          && quad.getTopRight().x() > 0 && quad.getTopRight().x() < image.dimensions.x()))
                  && (
                      // Check that the bottom is below the top of the screen and the top is below the bottom of
                      // the screen
                      (quad.getBottomRight().y() < image.dimensions.y() && quad.getTopRight().y() > 0)
                      || (quad.getBottomLeft().y() < image.dimensions.y() && quad.getTopLeft().y() > 0)))) {
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

    Eigen::Matrix<double, 3, 4> VirtualGoalPost::cameraSpaceGoalProjection(
        const Eigen::Affine2d& robotPose,
        const Eigen::Vector3d& goalLocation,
        const message::support::FieldDescription& field,
        const Eigen::Affine3d& Hgc,
        // camtoground is either camera to ground or camera to world, depending on application
        const bool& failIfNegative) {

        Eigen::Affine3d Hcf = getFieldToCam(robotPose, Hgc);

        // NOTE: this code assumes that goalposts are boxes with width and high of goalpost_diameter
        // make the base goal corners
        Eigen::Matrix4d goalTopCorners;
        goalTopCorners.row(3)                 = Eigen::Vector4d::Ones();
        goalTopCorners.topRows<3>().colwise() = goalLocation;
        goalTopCorners.topRows<1>() -= Eigen::Vector4d::Constant(0.5 * field.dimensions.goalpost_width);
        goalTopCorners.block<2, 1>(0, 0) += Eigen::Vector2d::Constant(field.dimensions.goalpost_width);
        goalTopCorners.block<2, 1>(1, 1) += Eigen::Vector2d::Constant(field.dimensions.goalpost_width);

        // We create camera world by using camera-torso -> torso-world -> world->field
        // transform the goals from field to camera
        Eigen::Matrix<double, 3, 4> goalBaseCorners = (Hcf * goalBaseCorners).topRows<3>();

        // if the goals are not in front of us, do not return valid normals
        if (failIfNegative && (goalBaseCorners.topRows<1>().array() < 0.0).any()) {
            return Eigen::Matrix<double, 3, 4>::Zero();
        }

        goalTopCorners.row(2).setConstant(field.goalpost_top_height);
        Eigen::Matrix<double, 3, 4> new_goalTopCorners = (Hcf * goalTopCorners).topRows<3>();

        // Select the (tl, tr, bl, br) corner points for normals
        Eigen::Vector4i cornerIndices = Eigen::Vector4i::Zero();

        Eigen::Vector4d pvals = goalBaseCorners.transpose() * goalBaseCorners.col(0).cross(new_goalTopCorners.col(0));
        Eigen::Vector4i baseIndices(1, 2, 3, 4);
        std::sort(baseIndices.data(), baseIndices.data() + baseIndices.size(), [&pvals](const int& a, const int& b) {
            return (pvals[a] < pvals[b]);
        });

        cornerIndices[2] = baseIndices[0];
        cornerIndices[3] = baseIndices[3];

        pvals = new_goalTopCorners.transpose() * goalBaseCorners.col(0).cross(new_goalTopCorners.col(0));
        Eigen::Vector4i topIndices(1, 2, 3, 4);
        std::sort(topIndices.data(), topIndices.data() + topIndices.size(), [&pvals](const int& a, const int& b) {
            return (pvals[a] < pvals[b]);
        });
        cornerIndices[0] = topIndices[0];
        cornerIndices[1] = topIndices[3];


        // Create the quad normal predictions. Order is Left, Right, Top, Bottom
        Eigen::Matrix<double, 3, 4> prediction;
        prediction.col(0) =
            goalBaseCorners.col(cornerIndices[2]).cross(new_goalTopCorners.col(cornerIndices[0])).normalized();
        prediction.col(1) =
            goalBaseCorners.col(cornerIndices[1]).cross(new_goalTopCorners.col(cornerIndices[3])).normalized();

        // for the top and bottom, we check the inner lines in case they are a better match (this stabilizes
        // observations and reflects real world)
        if (goalBaseCorners(2, baseIndices[0]) > goalBaseCorners(2, baseIndices[1])) {
            cornerIndices[2] = baseIndices[1];
        }
        if (goalBaseCorners(2, baseIndices[3]) > goalBaseCorners(2, baseIndices[2])) {
            cornerIndices[3] = baseIndices[2];
        }
        if (goalTopCorners(2, topIndices[0]) > new_goalTopCorners(2, topIndices[1])) {
            cornerIndices[0] = topIndices[1];
        }
        if (goalTopCorners(2, topIndices[3]) > new_goalTopCorners(2, topIndices[2])) {
            cornerIndices[1] = topIndices[2];
        }

        prediction.col(2) =
            new_goalTopCorners.col(cornerIndices[0]).cross(new_goalTopCorners.col(cornerIndices[1])).normalized();
        prediction.col(3) =
            goalBaseCorners.col(cornerIndices[3]).cross(goalBaseCorners.col(cornerIndices[2])).normalized();

        return prediction;
    }

    Eigen::Affine3d VirtualGoalPost::getFieldToCam(const Eigen::Affine2d& Tft,
                                                   // f = field
                                                   // t = torso
                                                   // c = camera
                                                   const Eigen::Affine3d& Htc) {

        Eigen::Affine3d Htf = Eigen::Affine3d::Identity();
        Htf                 = Htf.translate(Eigen::Vector3d(Tft.translation().x(), Tft.translation().y(), 0.0))
                  .rotate(Eigen::AngleAxisd(Eigen::Rotation2Dd(Tft.rotation()).angle(), Eigen::Vector3d::UnitZ()))
                  .inverse();

        return Htc.inverse() * Htf;
    }

    template <typename Scalar>
    Eigen::Matrix<Scalar, 2, 1> VirtualGoalPost::projectCamSpaceToScreen(const Eigen::Matrix<Scalar, 3, 1>& point,
                                                                         const message::input::Image::Lens& cam) {
        auto pinhole = [](const Eigen::Matrix<Scalar, 3, 1>& point,
                          const message::input::Image::Lens& cam) -> Eigen::Matrix<Scalar, 2, 1> {
            return Eigen::Matrix<Scalar, 2, 1>(static_cast<Scalar>(cam.focal_length) * point[1] / point[0],
                                               static_cast<Scalar>(cam.focal_length) * point[2] / point[0]);
        };

        auto radial = [](const Eigen::Matrix<Scalar, 3, 1>& point,
                         const message::input::Image::Lens& cam) -> Eigen::Matrix<Scalar, 2, 1> {
            Eigen::Matrix<Scalar, 3, 1> p = point.normalized();
            Scalar theta                  = std::acos(p.x());
            if (theta == 0) {
                return Eigen::Matrix<Scalar, 2, 1>::Zero();
            }
            Scalar r         = theta * cam.focal_length;
            Scalar sin_theta = std::sin(theta);
            Scalar px        = r * p.y() / sin_theta;
            Scalar py        = r * p.z() / sin_theta;

            return Eigen::Matrix<Scalar, 2, 1>(px, py) + cam.centre.cast<Scalar>();
        };

        switch (cam.projection.value) {
            case message::input::Image::Lens::Projection::RECTILINEAR: return pinhole(point, cam);
            case message::input::Image::Lens::Projection::EQUIDISTANT:
            case message::input::Image::Lens::Projection::EQUISOLID:  // TODO: do this properly
                return radial(point, cam);
            case message::input::Image::Lens::Projection::UNKNOWN:
            default: return Eigen::Matrix<Scalar, 2, 1>();
        }
    }

    template <typename Scalar>
    Eigen::Matrix<Scalar, 2, 1> VirtualGoalPost::screenToImage(const Eigen::Matrix<Scalar, 2, 1>& screen,
                                                               const Eigen::Matrix<unsigned int, 2, 1>& imageSize) {
        Eigen::Matrix<Scalar, 2, 1> v(static_cast<Scalar>(imageSize.x() - 1) * static_cast<Scalar>(0.5),
                                      static_cast<Scalar>(imageSize.y() - 1) * static_cast<Scalar>(0.5));
        v -= screen;
        return v.array().round().matrix();
    }

}  // namespace support
}  // namespace module
