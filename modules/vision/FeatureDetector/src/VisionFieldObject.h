/*
 * This file is part of NUBots FeatureDetector.
 *
 * NUBots FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_VISIONFIELDOBJECT_H
#define MODULES_VISION_VISIONFIELDOBJECT_H

#include <nuclear>
#include <armadillo>
#include <map>
#include <vector>
#include <string>
#include <utility>

#include "NUPoint.h"

namespace modules {
    namespace vision {

        enum DISTANCE_METHOD {
            WIDTH,
            D2P,
            AVERAGE,
            LEAST
        };

        //! VFO_ID enum and associated std::string conversion methods
        enum VFO_ID {
            BALL,
            FIELDLINE,
            CORNER,  
            CENTRE_CIRCLE,
            OBSTACLE,
            GOAL_L,
            GOAL_R,
            GOAL_U,  
            GOAL_Y_L,
            GOAL_Y_R,
            GOAL_Y_U,
            GOAL_B_L,
            GOAL_B_R,
            GOAL_B_U,
            INVALID
        };

        /**
         *   @author Shannon Fenn: shannon.fenn@uon.edu.au
         *   @brief Abstract parent class for internal representation of field objects.
         *   @note Ported to NUClear system by Alex Biddulph 17-12-2013
         */
        class VisionFieldObject {
        public:
            VisionFieldObject();
            virtual ~VisionFieldObject();

            VFO_ID getID() const;

            std::string getName() const;

            bool isValid() const;

            const NUPoint& getLocation() const;

            //! @brief returns the screen location in pixels (relative to the top left).
            arma::vec2 getLocationPixels() const;

            //! @brief returns the angular screen location (relative to the camera) in radians.
            arma::vec2 getLocationAngular() const;

            //! @brief returns the screen size in pixels.
            arma::vec2 getScreenSize() const;

            //! @brief returns the field position relative to the robot.
            virtual arma::vec3 getRelativeFieldCoords() const;

            virtual double findScreenError(VisionFieldObject* other) const = 0;
            virtual double findGroundError(VisionFieldObject* other) const = 0;

            //! @brief returns whether the given id maps to a goal
            static bool isGoal(VFO_ID id);

            //! @brief converts a VisionFieldObject Id into a string.
            static std::string VFOName(VFO_ID id);

            //! @brief converts a string into a VisionFieldObject Id.
            static VFO_ID VFOFromName(const std::string& name);

            static DISTANCE_METHOD getDistanceMethodFromName(const std::string& name);
            static std::string getDistanceMethodName(const DISTANCE_METHOD& method);

        protected:
            NUPoint m_location;                         //! @variable The location of the object (includes screen, radial and ground position).
            arma::vec2 m_sizeOnScreen;                  //! @variable The width and height on screen in pixels.

            VFO_ID m_id;
            float m_confidence;                         //! unused
            float m_error;                              //! unused
            arma::vec3 m_sphericalError;                //! @variable The error in each of the spherical dimensions.
            bool valid;                                 //! @variable Whether the object is valid.
            //bool distance_valid;                      //! @variable Whether the distance is valid.
        };

    }
}

#endif // MODULES_VISION_VISIONFIELDOBJECT_H
