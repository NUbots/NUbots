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
#include <nuclear>
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"
#include "utility/math/matrix.h"
#include <cmath>


namespace utility {
  namespace math {
    namespace vision {
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
        /*! @param ballVisualCentrePixels - centre of the ball onscreen in centre-zero coordinates (x=0,y=0 at the centre of the screen)
            @param ballVisualDiameterPixels
            @param ballWidth
            @param effectiveScreenDistancePixels
        */
        inline double widthBasedDistanceToBall(const arma::vec2& ballVisualCentrePixels,
                                               const double& ballVisualDiameterPixels,
                                               const double& ballWidth,
                                               const double& effectiveScreenDistancePixels)
        {
            double ballVisualDistance = arma::norm(ballVisualCentrePixels); 
            
            double ballVisualRadius = ballVisualDiameterPixels / 2.0;
            
            double ballCloseDistance = ballVisualDistance - ballVisualRadius;
            double ballFarDistance = ballVisualDistance + ballVisualRadius; 

            double thetaClose = std::atan2(ballCloseDistance, effectiveScreenDistancePixels);
            double thetaFar = std::atan2(ballFarDistance, effectiveScreenDistancePixels);

            double ballAngularWidth = thetaFar - thetaClose;

            return (ballWidth / 2.0) / std::tan(ballAngularWidth / 2.0);
        }

        /*! @brief Assumes goal is normal to camera view 
            @param goalBaseCentreVisualPixels - centre of the goal base onscreen in centre-zero coordinates (x=0,y=0 at the centre of the screen)
            @param effectiveScreenDistancePixels
        */
        inline double widthBasedDistanceToGoal(const arma::vec2& goalBaseCentreVisualPixels,
                                               const double& goalVisualDiameterPixels,
                                               const double& goalWidth,
                                               const double& effectiveScreenDistancePixels)
        {
            return widthBasedDistanceToBall({goalBaseCentreVisualPixels[0], 0},
                                             goalVisualDiameterPixels,
                                             goalWidth,
                                             effectiveScreenDistancePixels);
        }
    }
  }
}
#endif