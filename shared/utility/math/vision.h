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

#include <armadillo>
#include <cmath>
#include <nuclear>
#include "message/input/CameraParameters.h"
#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "utility/input/ServoID.h"
#include "utility/math/angle.h"
#include "utility/math/geometry/ParametricLine.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/eigen_armadillo.h"


namespace utility {
namespace math {
    namespace vision {

        using ServoID = utility::input::ServoID;


        // UNIVERSAL METHODS
        /*! @brief
            @param cam - coordinates in camera space of the pixel (cam[0] = y coordinate pixels, cam[1] = z coordinate
           pixels)
            @return im - coordinates on the screen in image space measured x across, y down, zero at top left
        */
        inline arma::ivec2 screenToImage(const arma::vec2& screen, const arma::uvec2& imageSize) {
            arma::vec2 v = arma::vec2({double(imageSize[0] - 1) * 0.5, double(imageSize[1] - 1) * 0.5}) - screen;
            return arma::ivec2({int(lround(v[0])), int(lround(v[1]))});
        }
        inline arma::vec2 screenToImageCts(const arma::vec2& screen, const arma::uvec2& imageSize) {
            return arma::vec2({double(imageSize[0] - 1) * 0.5, double(imageSize[1] - 1) * 0.5}) - screen;
        }
        inline arma::vec2 imageToScreen(const arma::ivec2& im, const arma::uvec2& imageSize) {
            return arma::vec2({double(imageSize[0] - 1) * 0.5, double(imageSize[1] - 1) * 0.5}) - im;
        }
        inline arma::vec2 imageToScreen(const arma::vec2& im, const arma::uvec2& imageSize) {
            return arma::vec2({double(imageSize[0] - 1) * 0.5, double(imageSize[1] - 1) * 0.5}) - im;
        }
        // END UNIVERSAL METHODS


        // Pinhole methods
        namespace pinhole {
            /*! @brief uses pinhole cam model
                @param point - Point in camera space (x along view axis, y to left of screen, z up along screen)
            */
            inline arma::vec2 projectCamSpaceToScreen(const arma::vec3& point,
                                                      const message::input::CameraParameters& cam) {
                const double& camFocalLengthPixels = cam.pinhole.focalLengthPixels;
                return {camFocalLengthPixels * point[1] / point[0], camFocalLengthPixels * point[2] / point[0]};
            }

            inline arma::vec3 getCamFromScreen(const arma::vec2& screen, const message::input::CameraParameters& cam) {
                const double& camFocalLengthPixels = cam.pinhole.focalLengthPixels;
                return arma::normalise(arma::vec3{camFocalLengthPixels, screen[0], screen[1]});
            }
        }  // namespace pinhole

        // Radial methods
        namespace radial {

            inline arma::vec2 projectCamSpaceToScreen(const arma::vec3& point,
                                                      const message::input::CameraParameters& cam) {
                arma::vec3 p = arma::normalise(point);
                float theta  = std::acos(p[0]);
                if (theta == 0) {
                    return arma::vec2({0, 0});
                }
                float r         = theta / cam.radial.radiansPerPixel;
                float sin_theta = std::sin(theta);
                float px        = r * p[1] / (sin_theta);
                float py        = r * p[2] / (sin_theta);

                return arma::vec2({px, py}) + arma::vec2({double(cam.centreOffset[0]), double(cam.centreOffset[1])});
            }

            inline arma::vec3 getCamFromScreen(const arma::vec2& p, const message::input::CameraParameters& cam) {
                arma::vec2 px = p - arma::vec2({double(cam.centreOffset[0]), double(cam.centreOffset[1])});
                float r       = std::sqrt(std::pow(px[0], 2) + std::pow(px[1], 2));
                if (r == 0) {
                    return {1, 0, 0};
                }
                float sx = (std::cos(cam.radial.radiansPerPixel * r));
                float sy = std::sin(cam.radial.radiansPerPixel * r) * (float(px[0]) / r);
                float sz = std::sin(cam.radial.radiansPerPixel * r) * (float(px[1]) / r);

                // Swizzle components so x is out of camera, y is to the left, z is up
                // Matches input of pointToPixel
                return arma::normalise(arma::vec3({sx, sy, sz}));
            }

        }  // namespace radial

        /////////////////////
        // Switch methods
        /////////////////////

        /*! @brief uses pinhole cam model
            @param point - Point in camera space (x along view axis, y to left of screen, z up along screen)
        */
        inline arma::vec2 projectCamSpaceToScreen(const arma::vec3& point,
                                                  const message::input::CameraParameters& cam) {
            switch (message::input::CameraParameters::LensType::Value(cam.lens)) {
                case (message::input::CameraParameters::LensType::PINHOLE):
                    return pinhole::projectCamSpaceToScreen(point, cam);
                case (message::input::CameraParameters::LensType::RADIAL):
                    return radial::projectCamSpaceToScreen(point, cam);
            }
            return arma::vec2();
        }

        inline arma::vec3 getCamFromScreen(const arma::vec2& screen, const message::input::CameraParameters& cam) {
            switch (message::input::CameraParameters::LensType::Value(cam.lens)) {
                case (message::input::CameraParameters::LensType::PINHOLE):
                    return pinhole::getCamFromScreen(screen, cam);
                case (message::input::CameraParameters::LensType::RADIAL): return radial::getCamFromScreen(screen, cam);
            }
            return arma::vec2();
        }
        /////////////////////
        // END SWITCH METHODS
        /////////////////////


        inline arma::vec3 getCamFromImage(const arma::ivec2& image, const message::input::CameraParameters& cam) {
            return getCamFromScreen(imageToScreen(image, convert<uint, 2>(cam.imageSizePixels)), cam);
        }

        inline arma::ivec2 getImageFromCam(const arma::vec3& unit_vector, const message::input::CameraParameters& cam) {
            return screenToImage(projectCamSpaceToScreen(unit_vector, cam), convert<uint, 2>(cam.imageSizePixels));
        }

        inline arma::vec2 getImageFromCamCts(const arma::vec3& unit_vector,
                                             const message::input::CameraParameters& cam) {
            return screenToImageCts(projectCamSpaceToScreen(unit_vector, cam), convert<uint, 2>(cam.imageSizePixels));
        }


        inline double getParallaxAngle(const arma::vec2& screen1,
                                       const arma::vec2& screen2,
                                       const message::input::CameraParameters& cam) {
            arma::vec3 camSpaceP1 = getCamFromScreen(screen1, cam);
            arma::vec3 camSpaceP2 = getCamFromScreen(screen2, cam);

            return utility::math::angle::acos_clamped(arma::dot(camSpaceP1, camSpaceP2)
                                                      / (arma::norm(camSpaceP1) * arma::norm(camSpaceP2)));
        }

        inline double widthBasedDistanceToCircle(const double& radius,
                                                 const arma::vec3& c1,
                                                 const arma::vec3& c2,
                                                 const message::input::CameraParameters&) {
            double parallaxAngle                = utility::math::angle::acos_clamped(arma::norm_dot(c1, c2));
            double correctionForClosenessEffect = radius * std::sin(parallaxAngle / 2.0);

            return radius / std::tan(parallaxAngle / 2.0) + correctionForClosenessEffect;
        }

        /*! @param separation - Known distance between points in camera space
            @param s1,s2 - Measured screen coordinates in pixels of points
            @param cam - Distance to the virtual camera screen in pixels
        */
        inline double distanceToEquidistantPoints(const double& separation,
                                                  const arma::vec2& s1,
                                                  const arma::vec2& s2,
                                                  const message::input::CameraParameters& cam) {
            double parallaxAngle = getParallaxAngle(s1, s2, cam);
            return (separation / 2) / std::tan(parallaxAngle / 2);
        }

        /*! @brief returns an estimate of the distance to two points which have a known separation.
            @param separation - Known distance between points in camera space
            @param cam1,cam2 - Measured camera space unit vectors point toward the points
        */
        inline double distanceToEquidistantCamPoints(const double& separation,
                                                     const arma::vec3& cam1,
                                                     const arma::vec3& cam2) {
            double parallaxAngle = utility::math::angle::acos_clamped(arma::norm_dot(cam1, cam2));
            return (separation / 2) / std::tan(parallaxAngle / 2);
        }


        inline arma::vec2 projectWorldPointToScreen(const arma::vec4& point,
                                                    const utility::math::matrix::Transform3D& camToGround,
                                                    const message::input::CameraParameters& cam) {
            arma::vec4 camSpacePoint = camToGround.i() * point;
            return projectCamSpaceToScreen(camSpacePoint.rows(0, 2), cam);
        }
        inline arma::vec2 projectWorldPointToScreen(const arma::vec3& point,
                                                    const utility::math::matrix::Transform3D& camToGround,
                                                    const message::input::CameraParameters& cam) {
            arma::vec4 point_ = arma::ones(4);
            point_.rows(0, 2) = point;
            return projectWorldPointToScreen(point_, camToGround, cam);
        }

        inline arma::vec3 projectCamToPlane(const arma::vec3& cam,
                                            const utility::math::matrix::Transform3D& camToGround,
                                            const utility::math::geometry::Plane<3>& plane) {
            arma::vec3 lineDirection = camToGround.submat(0, 0, 2, 2) * cam;
            arma::vec3 linePosition  = camToGround.submat(0, 3, 2, 3);

            utility::math::geometry::ParametricLine<3> line;
            line.setFromDirection(lineDirection, linePosition);

            return plane.intersect(line);
        }

        inline arma::vec3 getGroundPointFromScreen(const arma::vec2& screenPos,
                                                   const utility::math::matrix::Transform3D& camToGround,
                                                   const message::input::CameraParameters& cam) {
            return projectCamToPlane(
                getCamFromScreen(screenPos, cam), camToGround, utility::math::geometry::Plane<3>({0, 0, 1}, {0, 0, 0}));
        }

        inline double distanceToVerticalObject(const arma::vec2& top,
                                               const arma::vec2& base,
                                               const double& objectHeight,
                                               const double& robotHeight,
                                               const message::input::CameraParameters& cam) {

            // Parallax from top to base
            double theta = getParallaxAngle(top, base, cam);

            // The following equation comes from the dot product identity a*b = |a||b|cos(theta)
            // As we can calculate theta and |a||b| in terms of perpendicular distance to the object we can solve this
            // equation for an inverse equation. It may not be pretty but it will get the job done

            // Cos theta
            const double c  = cos(theta);
            const double c2 = c * c;
            // Object height
            const double H  = objectHeight;
            const double H2 = H * H;
            // Robot Height
            const double h  = robotHeight;
            const double h2 = h * h;

            double innerExpr =
                std::abs(c) * sqrt(H2 * (4.0 * H * h + H2 * c2 + 4.0 * h2 * c2 - 4.0 * h2 - 4.0 * H * h * c2));
            double divisor = 2 * std::abs(std::sin(theta));
            return M_SQRT2 * sqrt(2.0 * H * h + H2 * c2 + 2.0 * h2 * c2 + innerExpr - 2.0 * h2 - 2.0 * H * h * c2)
                   / divisor;
        }

        inline arma::vec objectDirectionFromScreenAngular(const arma::vec& screenAngular) {
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

        inline arma::vec screenAngularFromObjectDirection(const arma::vec& v) {
            return {std::atan2(v[1], v[0]), std::atan2(v[2], v[0])};
        }

        inline utility::math::matrix::Transform3D getFieldToCam(const utility::math::matrix::Transform2D& Tft,
                                                                // f = field
                                                                // t = torso
                                                                // c = camera
                                                                const utility::math::matrix::Transform3D& Htc) {

            // arma::vec3 rWFf;
            // rWFf.rows(0,1) = -Twf.rows(0,1);
            // rWFf[2] = 0.0;
            // // Hwf = rWFw * Rwf
            // utility::math::matrix::Transform3D Hwf =
            //     utility::math::matrix::Transform3D::createRotationZ(-Twf[2])
            //     * utility::math::matrix::Transform3D::createTranslation(rWFf);

            utility::math::matrix::Transform3D Htf = utility::math::matrix::Transform3D(Tft).i();

            return Htc.i() * Htf;
        }

        inline arma::mat::fixed<3, 4> cameraSpaceGoalProjection(
            const arma::vec3& robotPose,
            const arma::vec3& goalLocation,
            const message::support::FieldDescription& field,
            const utility::math::matrix::Transform3D& camToGround,
            const bool& failIfNegative =
                true)  // camtoground is either camera to ground or camera to world, depending on application
        {
            utility::math::matrix::Transform3D Hcf = getFieldToCam(robotPose, camToGround);
            // NOTE: this code assumes that goalposts are boxes with width and high of goalpost_diameter
            // make the base goal corners
            arma::mat goalBaseCorners(4, 4);
            goalBaseCorners.row(3).fill(1.0);
            goalBaseCorners.submat(0, 0, 2, 3).each_col() = goalLocation;
            goalBaseCorners.submat(0, 0, 1, 3) -= 0.5 * field.dimensions.goalpost_width;
            goalBaseCorners.submat(0, 0, 1, 0) += field.dimensions.goalpost_width;
            goalBaseCorners.submat(1, 1, 2, 1) += field.dimensions.goalpost_width;

            // make the top corner points
            arma::mat goalTopCorners = goalBaseCorners;


            // We create camera world by using camera-torso -> torso-world -> world->field
            // transform the goals from field to camera
            goalBaseCorners = arma::mat(Hcf * goalBaseCorners).rows(0, 2);


            // if the goals are not in front of us, do not return valid normals
            arma::mat::fixed<3, 4> prediction;
            if (failIfNegative and arma::any(goalBaseCorners.row(0) < 0.0)) {
                prediction.fill(0);
                return prediction;
            }

            goalTopCorners.row(2).fill(field.goalpost_top_height);
            goalTopCorners = arma::mat(Hcf * goalTopCorners).rows(0, 2);

            // Select the (tl, tr, bl, br) corner points for normals
            arma::ivec4 cornerIndices;
            cornerIndices.fill(0);


            arma::vec pvals        = goalBaseCorners.t() * arma::cross(goalBaseCorners.col(0), goalTopCorners.col(0));
            arma::uvec baseIndices = arma::sort_index(pvals);
            cornerIndices[2]       = baseIndices[0];
            cornerIndices[3]       = baseIndices[3];


            pvals                 = goalTopCorners.t() * arma::cross(goalBaseCorners.col(0), goalTopCorners.col(0));
            arma::uvec topIndices = arma::sort_index(pvals);
            cornerIndices[0]      = topIndices[0];
            cornerIndices[1]      = topIndices[3];


            // Create the quad normal predictions. Order is Left, Right, Top, Bottom

            prediction.col(0) = arma::normalise(
                arma::cross(goalBaseCorners.col(cornerIndices[2]), goalTopCorners.col(cornerIndices[0])));
            prediction.col(1) = arma::normalise(
                arma::cross(goalBaseCorners.col(cornerIndices[1]), goalTopCorners.col(cornerIndices[3])));

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


            prediction.col(2) = arma::normalise(
                arma::cross(goalTopCorners.col(cornerIndices[0]), goalTopCorners.col(cornerIndices[1])));
            prediction.col(3) = arma::normalise(
                arma::cross(goalBaseCorners.col(cornerIndices[3]), goalBaseCorners.col(cornerIndices[2])));

            return prediction;
        }

    }  // namespace vision
}  // namespace math
}  // namespace utility

#endif
