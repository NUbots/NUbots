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

#ifndef MODULES_VISION_VISIONKINEMATICS_H
#define MODULES_VISION_VISIONKINEMATICS_H

#include <nuclear>
#include <armadillo>
#include <vector>
#include <iostream>

#include "utility/math/matrix.h"
#include "utility/math/coordinates.h"
#include "VisionFieldObject.h"
#include "messages/input/Sensors.h"

#include "NUPoint.h"

namespace modules {
    namespace vision {

        class VisionKinematics {
        public:
            VisionKinematics();

            void setParameters(float RADIAL_CORRECTION_COEFFICIENT_,
                               float SCREEN_LOCATION_UNCERTAINTY_PIXELS_
                                );

            // 2D distortion transform.
            arma::vec2 correctDistortion(const arma::vec2& point);

            void calculateRepresentationsFromPixelLocation(NUPoint& pt, bool knownDistance = false, double val = 0.0) const;
            void calculateRepresentationsFromPixelLocation(std::vector<NUPoint>& pts, bool knownDistance = false, double val = 0.0) const;
     
            double getCameraDistanceInPixels() const;

            arma::vec2 getFOV() const;

            arma::vec2 getImageSize() const{
                return m_imageSize;
            }

            double getD2PError(const NUPoint& location) const;

            //! Calculate the field of view and effective camera distance in pixels.
            void setCamParams(arma::vec2 imagesize, arma::vec2 fov);

            void setSensors(const messages::input::Sensors& sensors);

            arma::vec3 calculateSphericalError(NUPoint location, DISTANCE_METHOD distanceMethod, float width) const;

        private:
            // void preCalculateTransforms();

            void screenToRadial3D(NUPoint &point, double distance) const;
            NUPoint screenToRadial3D(const arma::vec2& point, double distance) const;

            /**
              * Calculates the distance to a point at a given height
              * @param pixel The pixel location in the image relative to the top left of the screen.
              * @param object_height The height of the point to be measured (from the ground).
              * @return A 3 dimensional vector containing the distance, bearing and elevation to the point.
              */
            arma::vec3 distanceToPoint(arma::vec2 pixel, double objectHeight = 0.0) const;

        private:
            arma::vec2 m_FOV;               //!FOV in radians horizontally and vertically
            double m_effectiveCameraDistancePixels;

            arma::vec2 m_imageSize;
            arma::vec2 m_imageCentre;
            arma::vec2 m_tanHalfFOV;
            arma::vec2 m_screenToRadialFactor;

            // New for transforms.
            
            arma::mat44 m_camToBodyMatrix;    
            float m_bodyHeight;       

            float RADIAL_CORRECTION_COEFFICIENT;
            float SCREEN_LOCATION_UNCERTAINTY_PIXELS;
        };

    }
}
#endif // MODULES_VISION_VISIONKINEMATICS_H
