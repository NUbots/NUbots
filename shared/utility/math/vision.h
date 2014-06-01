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
        inline arma::mat44 calculateWorldToCameraTransform(const messages::input::Sensors& sensors, const messages::localisation::Self& self){
            arma::vec2 selfHeading = arma::normalise(self.heading);
            arma::mat44 robotToWorld_world;
            robotToWorld_world <<  selfHeading[0]  <<  -selfHeading[1]  <<  0 <<      self.position[0] << arma::endr
                               <<  selfHeading[1]  <<   selfHeading[0]  <<  0 <<      self.position[1] << arma::endr
                               <<               0  <<                0  <<  1 <<  sensors.bodyCentreHeight << arma::endr
                               <<               0  <<                0  <<  0 <<                                 1;


            arma::mat44 cameraToBody_body = sensors.forwardKinematics.at(messages::input::ServoID::HEAD_PITCH);

            arma::mat44 robotToBody_body = arma::eye(4,4);
            //TODO: copy localisation in develop
            robotToBody_body.submat(0,0,2,2) = sensors.orientation;
            auto worldToCamera_camera = utility::math::matrix::orthonormal44Inverse(cameraToBody_body) * robotToBody_body * utility::math::matrix::orthonormal44Inverse(robotToWorld_world);
            NUClear::log("\nrobotToWorld_world\n", robotToWorld_world,"\nrobotToBody_body\n", robotToBody_body,"\nworldToCamera_camera\n", worldToCamera_camera);
            return worldToCamera_camera;
        }


        inline arma::vec4 directionVectorFromScreenAngular(const arma::vec2& screenAngular){
          double theta = screenAngular[0];
          double phi = screenAngular[1];
          return {std::cos(theta) * std::cos(phi), std::sin(theta) * std::cos(phi), std::sin(phi), 0};
        }
        /*! Uses vec for backwards compatibility with 3d homogeneous coordinates
        */
        inline arma::vec2 screenAngularFromDirectionVector(const arma::vec& direction){
          return { std::atan2(direction[1], direction[0]) , std::atan2(direction[2], arma::norm(arma::vec2({direction[0], direction[1]})))};
        }

        inline arma::vec2 rotateAngularDirection(const arma::vec2& screenAngular, const arma::mat44& R){                  
            return screenAngularFromDirectionVector(R*directionVectorFromScreenAngular(screenAngular));
        }
    }
  }
}
#endif