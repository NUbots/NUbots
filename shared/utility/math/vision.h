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

#include <cmath>
#include <armadillo>
#include <nuclear>
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"
#include "utility/math/matrix.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/geometry/ParametricLine.h"

namespace utility {
namespace math {
namespace vision {

    /**************************************************************
     *SLAME STUFF: TO BE REMOVED FOR GOOD STUFF BELOW (DO NOT USE)*
     **************************************************************/
    /*! @brief Calculates the transformation for taking homogeneous points from world coordinates to camera coordinates
    */
    inline arma::mat calculateWorldToCameraTransform(const messages::input::Sensors& sensors, const messages::localisation::Self& self){
        arma::vec selfHeading = arma::normalise(self.heading);
        arma::mat robotToWorld_world;
        robotToWorld_world <<  selfHeading[0]  <<  -selfHeading[1]  <<  0 <<      self.position[0] << arma::endr
                           <<  selfHeading[1]  <<   selfHeading[0]  <<  0 <<      self.position[1] << arma::endr
                           <<               0  <<                0  <<  1 <<  sensors.bodyCentreHeight << arma::endr
                           <<               0  <<                0  <<  0 <<                                 1;

        arma::mat cameraToBody_body = sensors.forwardKinematics.at(messages::input::ServoID::HEAD_PITCH);
        arma::mat robotToBody_body = arma::eye(4,4);
        robotToBody_body.submat(0,0,2,2) = sensors.orientation;

        auto worldToCamera_camera = utility::math::matrix::orthonormal44Inverse(cameraToBody_body) * robotToBody_body * utility::math::matrix::orthonormal44Inverse(robotToWorld_world);
        //Confirmed to be correct by Jake Fountain 2014
        return worldToCamera_camera;
    }


    inline arma::vec directionVectorFromScreenAngular(const arma::vec& screenAngular){
        double theta = screenAngular[0];
        double phi = screenAngular[1];
        return {std::cos(theta) * std::cos(phi), std::sin(theta) * std::cos(phi), std::sin(phi), 0};
    }
    /*! Uses vec for backwards compatibility with 3d homogeneous coordinates
    */
    inline arma::vec screenAngularFromDirectionVector(const arma::vec& direction){
        return { std::atan2(direction[1], direction[0]) , std::atan2(direction[2], arma::norm(arma::vec({direction[0], direction[1]})))};
    }

    inline arma::vec rotateAngularDirection(const arma::vec& screenAngular, const arma::mat& R){
        return screenAngularFromDirectionVector(R*directionVectorFromScreenAngular(screenAngular));
    }

    inline arma::vec screenPositionFromDirectionVector(const arma::vec& directionVector){
        return {directionVector[1] / directionVector[0], directionVector[2] / directionVector[0]};
    }

    inline arma::vec screenPositionFromScreenAngular(const arma::vec& screenAngular){
        return screenPositionFromDirectionVector(directionVectorFromScreenAngular(screenAngular));
    }

    /************************************************
     *                 GOOD STUFF                   *
     ************************************************/

    inline double getParallaxAngle(const arma::vec2& screen1, const arma::vec2& screen2, const double& camFocalLengthPixels){
        arma::vec3 camSpaceP1 = {camFocalLengthPixels, screen1[0], screen1[1]};
        arma::vec3 camSpaceP2 = {camFocalLengthPixels, screen2[0], screen2[1]};

        return std::acos(arma::dot(camSpaceP1,camSpaceP2) / (arma::norm(camSpaceP1) * arma::norm(camSpaceP2)));
    }

    inline double widthBasedDistanceToCircle(const double& circleDiameter, const arma::vec2& s1, const arma::vec2& s2, const double& camFocalLengthPixels){
        double parallaxAngle = getParallaxAngle(s1, s2, camFocalLengthPixels);
        double correctionForClosenessEffect = 0.5 * circleDiameter * std::cos((M_PI - parallaxAngle) / 2.0);

        return (circleDiameter / 2) / std::tan(parallaxAngle / 2) + correctionForClosenessEffect;
    }

    /*! @param separation - Known distance between points in camera space
        @param s1,s2 - Measured screen coordinates in pixels of points
        @param camFocalLengthPixels - Distance to the virtual camera screen in pixels
    */
    inline double distanceToEquidistantPoints(const double& separation, const arma::vec2& s1, const arma::vec2& s2, const double& camFocalLengthPixels){
        double parallaxAngle = getParallaxAngle(s1, s2, camFocalLengthPixels);
        return (separation / 2) / std::tan(parallaxAngle / 2);
    }

    /*! @brief
        @param cam - coordinates in camera space of the pixel (cam[0] = y coordinate pixels, cam[1] = z coordinate pixels)
        @return im - coordinates on the screen in image space measured x across, y down, zero at top left
    */
    inline arma::ivec2 screenToImage(const arma::vec2& screen, const arma::uvec2& imageSize){
        arma::vec2 v = arma::vec2({ double(imageSize[0] - 1) * 0.5, double(imageSize[1] - 1) * 0.5 }) - screen;
        return arma::ivec2({ int(lround(v[0])), int(lround(v[1])) });
    }
    inline arma::vec2 imageToScreen(const arma::ivec2& im, const arma::uvec2& imageSize){
        return arma::vec2({ double(imageSize[0] - 1) * 0.5, double(imageSize[1] - 1) * 0.5 }) - im;
    }
    inline arma::vec2 imageToScreen(const arma::vec2& im, const arma::uvec2& imageSize){
        return arma::vec2({ double(imageSize[0] - 1) * 0.5, double(imageSize[1] - 1) * 0.5 }) - im;
    }

    /*! @brief uses pinhole cam model
        @param point - Point in camera space (x along view axis, y to left of screen, z up along screen)
    */
    inline arma::vec2 projectCamSpaceToScreen(const arma::vec3& point, const double& camFocalLengthPixels){
        return {camFocalLengthPixels * point[1] / point[0], camFocalLengthPixels * point[2] / point[0]};
    }

    inline arma::vec2 projectWorldPointToScreen(const arma::vec4& point, const arma::mat44& camToGround, const double& camFocalLengthPixels){
        arma::vec4 camSpacePoint = utility::math::matrix::orthonormal44Inverse(camToGround) * point;
        return projectCamSpaceToScreen(camSpacePoint.rows(0,2), camFocalLengthPixels);
    }
    inline arma::vec2 projectWorldPointToScreen(const arma::vec3& point, const arma::mat44& camToGround, const double& camFocalLengthPixels){
        arma::vec4 point_ = arma::ones(4);
        point_.rows(0,2) = point;
        return projectWorldPointToScreen(point_, camToGround, camFocalLengthPixels);
    }

    inline arma::vec3 getCamFromScreen(const arma::vec2& screen, const double& camFocalLengthPixels){
        return arma::vec3{camFocalLengthPixels, screen[0], screen[1]};
    }

    inline arma::vec3 projectCamToGroundPlane(const arma::vec3& cam, const arma::mat44& camToGround){
        arma::vec3 lineDirection = camToGround.submat(0,0,2,2) * cam;
        arma::vec3 linePosition = camToGround.submat(0,3,2,3);

        utility::math::geometry::ParametricLine<3> line;
        line.setFromDirection(lineDirection, linePosition);

        utility::math::geometry::Plane<3> p;
        p.setFromNormal({0,0,1},{0,0,0});

        return p.intersect(line);
    }

    inline arma::vec3 getGroundPointFromScreen(const arma::vec2& screenPos, const arma::mat44& camToGround, const double& camFocalLengthPixels){
        return projectCamToGroundPlane(getCamFromScreen(screenPos, camFocalLengthPixels), camToGround);
    }


}
}
}

#endif