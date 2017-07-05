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
#ifndef UTILITY_MATH_VISION_H
#define UTILITY_MATH_VISION_H

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <nuclear>

#include "message/input/Sensors.h"
#include "message/localisation/FieldObject.h"
#include "message/support/FieldDescription.h"

#include "utility/input/ServoID.h"
#include "utility/math/angle.h"
#include "utility/math/geometry/ParametricLine.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"

namespace utility {
namespace math {
    namespace vision {

        using ServoID = utility::input::ServoID;

        inline double getParallaxAngle(const Eigen::Vector2d& screen1,
                                       const Eigen::Vector2d& screen2,
                                       const double& camFocalLengthPixels) {
            Eigen::Vector3d camSpaceP1 = {camFocalLengthPixels, screen1[0], screen1[1]};
            Eigen::Vector3d camSpaceP2 = {camFocalLengthPixels, screen2[0], screen2[1]};

            return utility::math::angle::acos_clamped(camSpaceP1.dot(camSpaceP2)
                                                      / (camSpaceP1.norm() * camSpaceP2.norm()));
        }

        inline double widthBasedDistanceToCircle(const double& radius,
                                                 const Eigen::Vector2d& s1,
                                                 const Eigen::Vector2d& s2,
                                                 const double& camFocalLengthPixels) {
            double parallaxAngle                = getParallaxAngle(s1, s2, camFocalLengthPixels);
            double correctionForClosenessEffect = radius * std::sin(parallaxAngle * 0.5);

            return radius / std::tan(parallaxAngle * 0.5) + correctionForClosenessEffect;
        }

        /*! @param separation - Known distance between points in camera space
            @param s1,s2 - Measured screen coordinates in pixels of points
            @param camFocalLengthPixels - Distance to the virtual camera screen in pixels
        */
        inline double distanceToEquidistantPoints(const double& separation,
                                                  const Eigen::Vector2d& s1,
                                                  const Eigen::Vector2d& s2,
                                                  const double& camFocalLengthPixels) {
            double parallaxAngle = getParallaxAngle(s1, s2, camFocalLengthPixels);
            return (separation * 0.5) / std::tan(parallaxAngle * 0.5);
        }

        /*! @brief
            @param cam - coordinates in camera space of the pixel (cam[0] = y coordinate pixels, cam[1] = z coordinate
           pixels)
            @return im - coordinates on the screen in image space measured x across, y down, zero at top left
        */
        inline Eigen::Vector2i screenToImage(const Eigen::Vector2d& screen,
                                             const Eigen::Matrix<unsigned int, 2, 1>& imageSize) {
            Eigen::Vector2d v{double(imageSize[0] - 1) * 0.5 - screen[0], double(imageSize[1] - 1) * 0.5 - screen[1]};
            return {int(lround(v[0])), int(lround(v[1]))};
        }
        inline Eigen::Vector2d imageToScreen(const Eigen::Vector2i& im,
                                             const Eigen::Matrix<unsigned int, 2, 1>& imageSize) {
            return {double(imageSize[0] - 1) * 0.5 - im[0], double(imageSize[1] - 1) * 0.5 - im[1]};
        }
        inline Eigen::Vector2d imageToScreen(const Eigen::Vector2d& im,
                                             const Eigen::Matrix<unsigned int, 2, 1>& imageSize) {
            return {double(imageSize[0] - 1) * 0.5 - im[0], double(imageSize[1] - 1) * 0.5 - im[1]};
        }

        /*! @brief uses pinhole cam model
            @param point - Point in camera space (x along view axis, y to left of screen, z up along screen)
        */
        inline Eigen::Vector2d projectCamSpaceToScreen(const Eigen::Vector3d& point,
                                                       const double& camFocalLengthPixels) {
            return {camFocalLengthPixels * point[1] / point[0], camFocalLengthPixels * point[2] / point[0]};
        }

        inline Eigen::Vector2d projectWorldPointToScreen(const Eigen::Vector4d& point,
                                                         const utility::math::matrix::Transform3D& camToGround,
                                                         const double& camFocalLengthPixels) {
            Eigen::Vector4d camSpacePoint = camToGround.i() * point;
            return projectCamSpaceToScreen(camSpacePoint.head<3>(), camFocalLengthPixels);
        }
        inline Eigen::Vector2d projectWorldPointToScreen(const Eigen::Vector3d& point,
                                                         const utility::math::matrix::Transform3D& camToGround,
                                                         const double& camFocalLengthPixels) {
            Eigen::Vector4d point_ = Eigen::Vector4d::Ones();
            point_.head<3>()       = point;
            return projectWorldPointToScreen(point_, camToGround, camFocalLengthPixels);
        }

        inline Eigen::Vector3d getCamFromScreen(const Eigen::Vector2d& screen, const double& camFocalLengthPixels) {
            return {camFocalLengthPixels, screen[0], screen[1]};
        }

        inline Eigen::Vector3d projectCamToPlane(const Eigen::Vector3d& cam,
                                                 const utility::math::matrix::Transform3D& camToGround,
                                                 const utility::math::geometry::Plane<3>& plane) {
            Eigen::Vector3d lineDirection = camToGround.topLeftCorner<3, 3>() * cam;
            Eigen::Vector3d linePosition  = camToGround.topRightCorner<3, 1>();

            utility::math::geometry::ParametricLine<3> line;
            line.setFromDirection(lineDirection, linePosition);

            return plane.intersect(line);
        }

        inline Eigen::Vector3d getGroundPointFromScreen(const Eigen::Vector2d& screenPos,
                                                        const utility::math::matrix::Transform3D& camToGround,
                                                        const double& camFocalLengthPixels) {
            return projectCamToPlane(getCamFromScreen(screenPos, camFocalLengthPixels),
                                     camToGround,
                                     utility::math::geometry::Plane<3>({0, 0, 1}, {0, 0, 0}));
        }

        inline double distanceToVerticalObject(const Eigen::Vector2d& top,
                                               const Eigen::Vector2d& base,
                                               const double& objectHeight,
                                               const double& robotHeight,
                                               const double& camFocalLengthPixels) {

            // Parallax from top to base
            double theta = getParallaxAngle(top, base, camFocalLengthPixels);

            // The following equation comes from the dot product identity a*b = |a||b|cos(theta)
            // As we can calculate theta and |a||b| in terms of perpendicular distance to the object we can solve this
            // equation
            // for an inverse equation. It may not be pretty but it will get the job done

            // Cos theta
            const double c  = std::cos(theta);
            const double c2 = c * c;
            // Object height
            const double H  = objectHeight;
            const double H2 = H * H;
            // Robot Height
            const double h  = robotHeight;
            const double h2 = h * h;

            double innerExpr =
                std::abs(c) * std::sqrt(H2 * (4.0 * H * h + H2 * c2 + 4.0 * h2 * c2 - 4.0 * h2 - 4.0 * H * h * c2));
            double divisor = 2 * std::abs(std::sin(theta));
            return M_SQRT2 * sqrt(2.0 * H * h + H2 * c2 + 2.0 * h2 * c2 + innerExpr - 2.0 * h2 - 2.0 * H * h * c2)
                   / divisor;
        }

        inline Eigen::Vector3d objectDirectionFromScreenAngular(const Eigen::Vector2d& screenAngular) {
            if (std::fmod(std::fabs(screenAngular[0]), M_PI) == M_PI_2
                || std::fmod(std::fabs(screenAngular[1]), M_PI) == M_PI_2) {
                return {0, 0, 0};
            }
            double tanTheta        = std::tan(screenAngular[0]);
            double tanPhi          = std::tan(screenAngular[1]);
            double x               = 0;
            double y               = 0;
            double z               = 0;
            double denominator_sqr = 1 + tanTheta * tanTheta + tanPhi * tanPhi;
            // Assume facing forward st x>0 (which is fine for screen angular)
            x = 1 / std::sqrt(denominator_sqr);
            y = x * tanTheta;
            z = x * tanPhi;

            return {x, y, z};
        }

        inline Eigen::Vector2d screenAngularFromObjectDirection(const Eigen::Vector3d& v) {
            return {std::atan2(v[1], v[0]), std::atan2(v[2], v[0])};
        }

        inline utility::math::matrix::Transform3D getFieldToCam(const utility::math::matrix::Transform2D& Tft,
                                                                // f = field
                                                                // t = torso
                                                                // c = camera
                                                                const utility::math::matrix::Transform3D& Htc

                                                                ) {

            // Eigen::Vector3d rWFf;
            // rWFf.rows(0,1) = -Twf.rows(0,1);
            // rWFf[2] = 0.0;
            // // Hwf = rWFw * Rwf
            // utility::math::matrix::Transform3D Hwf =
            //     utility::math::matrix::Transform3D::createRotationZ(-Twf[2])
            //     * utility::math::matrix::Transform3D::createTranslation(rWFf);

            utility::math::matrix::Transform3D Htf = utility::math::matrix::Transform3D(Tft).inverse();

            return Htc.i() * Htf;
        }

        template <typename Scalar, int N>
        inline Eigen::Matrix<Scalar, N, 1> sort(const Eigen::Matrix<Scalar, N, 1>& in) {
            Eigen::Matrix<Scalar, N, 1> out{in};

            std::sort(out.data(), out.data() + N);
            return out;
        }

        template <typename Scalar, int N>
        inline Eigen::Matrix<Eigen::Index, N, 1> sort_index(const Eigen::Matrix<Scalar, N, 1>& in) {
            Eigen::Matrix<Eigen::Index, N, 1> out = Eigen::Matrix<Eigen::Index, N, 1>::LinSpaced(N, 0, N - 1);

            // sort indexes based on comparing values in v
            std::sort(out.data(), out.data() + N, [&in](const Eigen::Index& i1, const Eigen::Index& i2) {
                return in[i1] < in[i2];
            });

            return out;
        }

        // camtoground is either camera to ground or camera to world, depending on application
        inline Eigen::Matrix<double, 3, 4> cameraSpaceGoalProjection(
            const Eigen::Vector3d& robotPose,
            const Eigen::Vector3d& goalLocation,
            const message::support::FieldDescription& field,
            const utility::math::matrix::Transform3D& camToGround,
            const bool& failIfNegative = true) {
            utility::math::matrix::Transform3D Hcf = getFieldToCam(robotPose, camToGround);

            // NOTE: this code assumes that goalposts are boxes with width and high of goalpost_diameter
            // make the base goal corners
            Eigen::Matrix4d goalBaseCorners = Eigen::Matrix4d::Ones();
            goalBaseCorners.topLeftCorner<3, 4>().colwise() = goalLocation;
            goalBaseCorners.topLeftCorner<2, 4>().colwise().rowwise() -= 0.5 * field.dimensions.goalpost_diameter;
            goalBaseCorners.topLeftCorner<2, 1>().colwise().rowwise() += field.dimensions.goalpost_diameter;
            goalBaseCorners.block<2, 1>(1, 1).colwise().rowwise() += field.dimensions.goalpost_diameter;

            // make the top corner points
            Eigen::Matrix4d goalTopCorners = goalBaseCorners;

            // We create camera world by using camera-torso -> torso-world -> world->field
            // transform the goals from field to camera
            goalBaseCorners = Hcf * goalBaseCorners;

            // if the goals are not in front of us, do not return valid normals
            Eigen::Matrix<double, 3, 4> prediction = Eigen::Matrix<double, 3, 4>::Zero();
            if (failIfNegative && (goalBaseCorners.topRows<1>().array() < 0.0).any()) {
                return prediction;
            }

            goalTopCorners.middleRows<1>(2).fill(field.goalpost_top_height);
            goalTopCorners = Hcf * goalTopCorners;

            // Select the (tl, tr, bl, br) corner points for normals
            Eigen::Vector4i cornerIndices = Eigen::Vector4i::Zero();

            Eigen::Vector4d pvals =
                goalBaseCorners.topRows<3>().transpose()
                * goalBaseCorners.topRows<3>().leftCols<1>().cross(goalTopCorners.topRows<3>().leftCols<1>());
            Eigen::Matrix<Eigen::Index, 4, 1> baseIndices = sort_index(pvals);
            cornerIndices[2] = baseIndices[0];
            cornerIndices[3] = baseIndices[3];

            pvals = goalTopCorners.topRows<3>().transpose()
                    * goalBaseCorners.topRows<3>().leftCols<1>().cross(goalTopCorners.topRows<3>().leftCols<1>());
            Eigen::Matrix<Eigen::Index, 4, 1> topIndices = sort_index(pvals);
            cornerIndices[0] = topIndices[0];
            cornerIndices[1] = topIndices[3];

            // Create the quad normal predictions. Order is Left, Right, Top, Bottom
            prediction.col(0) = goalBaseCorners.topRows<3>()
                                    .col(cornerIndices[2])
                                    .cross(goalTopCorners.topRows<3>().col(cornerIndices[0]))
                                    .normalized();
            prediction.col(1) = goalBaseCorners.topRows<3>()
                                    .col(cornerIndices[1])
                                    .cross(goalTopCorners.topRows<3>().col(cornerIndices[3]))
                                    .normalized();

            // for the top and bottom, we check the inner lines in case they are a better match (this stabilizes
            // observations and reflects real world)
            if (goalBaseCorners(2, baseIndices[0]) > goalBaseCorners(2, baseIndices[1])) {
                cornerIndices[2] = baseIndices[1];
            }
            if (goalBaseCorners(2, baseIndices[3]) > goalBaseCorners(2, baseIndices[2])) {
                cornerIndices[3] = baseIndices[2];
            }
            if (goalTopCorners(2, topIndices[0]) > goalTopCorners(2, topIndices[1])) {
                cornerIndices[0] = topIndices[1];
            }
            if (goalTopCorners(2, topIndices[3]) > goalTopCorners(2, topIndices[2])) {
                cornerIndices[1] = topIndices[2];
            }

            prediction.col(2) = goalTopCorners.topRows<3>()
                                    .col(cornerIndices[0])
                                    .cross(goalTopCorners.topRows<3>().col(cornerIndices[1]))
                                    .normalized();
            prediction.col(3) = goalBaseCorners.topRows<3>()
                                    .col(cornerIndices[3])
                                    .cross(goalBaseCorners.topRows<3>().col(cornerIndices[2]))
                                    .normalized();

            return prediction;
        }
    }
}
}

#endif
