/*
 * This file is part of NUBots Utility.
 *
 * NUBots Utility is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots Utility is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots Utility.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_VISION_VISIONFIELDOBJECT_H
#define UTILITY_VISION_VISIONFIELDOBJECT_H

#include <nuclear>
#include <armadillo>
#include <map>
#include <vector>
#include <string>
#include <utility>

namesapce utility {
    namesapce vision {

        enum DistanceMethod {
            Width,
            D2P,
            Average,
            Least
        };

        //! VFO_ID enum and associated std::string conversion methods
        enum VFO_ID {
            BALL            = 0,
            FIELDLINE       = 1,
            CORNER          = 2,
            CENTRE_CIRCLE   = 3,
            OBSTACLE        = 4,
            GOAL_L          = 5,
            GOAL_R          = 6,
            GOAL_U          = 7,
            GOAL_Y_L        = 8,
            GOAL_Y_R        = 9,
            GOAL_Y_U        = 10,
            GOAL_B_L        = 11,
            GOAL_B_R        = 12,
            GOAL_B_U        = 13,
            INVALID         = 14
        };

        
        class VFOMap : public std::map<VFO_ID, std::pair<int, std::string>> {
        public:
            VFOMap() {
                (*this)[BALL]           = std::pair<int, std::string>(0, "BALL");
                (*this)[FIELDLINE]      = std::pair<int, std::string>(1, "FIELDLINE");
                (*this)[CORNER]         = std::pair<int, std::string>(2, "CORNER");
                (*this)[CENTRE_CIRCLE]  = std::pair<int, std::string>(3, "CENTRE_CIRCLE");
                (*this)[OBSTACLE]       = std::pair<int, std::string>(4, "OBSTACLE");
                (*this)[GOAL_L]         = std::pair<int, std::string>(5, "GOAL_L");
                (*this)[GOAL_R]         = std::pair<int, std::string>(6, "GOAL_R");
                (*this)[GOAL_U]         = std::pair<int, std::string>(7, "GOAL_U");
                (*this)[GOAL_Y_L]       = std::pair<int, std::string>(5, "GOAL_Y_L");
                (*this)[GOAL_Y_R]       = std::pair<int, std::string>(6, "GOAL_Y_R");
                (*this)[GOAL_Y_U]       = std::pair<int, std::string>(7, "GOAL_Y_U");
                (*this)[GOAL_B_L]       = std::pair<int, std::string>(5, "GOAL_B_L");
                (*this)[GOAL_B_R]       = std::pair<int, std::string>(6, "GOAL_B_R");
                (*this)[GOAL_B_U]       = std::pair<int, std::string>(7, "GOAL_B_U");
            }
        };

        VFOMap vfomap;

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
            static bool isGoal(VFO_ID id) const;

            //! @brief converts a VisionFieldObject Id into a string.
            static std::string VFOName(VFO_ID id);

            //! @brief converts a string into a VisionFieldObject Id.
            static VFO_ID VFOFromName(const std::string &name);

            //! @brief converts an int into a VisionFieldObject Id.
            static VFO_ID VFOFromInt(int n);

            //! @brief converts a VisionFieldObject Id into an int.
            static int intFromVFO(VFO_ID id);

            static int numVFOIDs();

            static DistanceMethod getDistanceMethodFromName(std::string name);
            static std::string getDistanceMethodName(DistanceMethod method);
            static std::string getLineMethodName(LineDetectionMethod method);
            static std::string getGoalMethodName(GoalDetectionMethod method);    

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

#endif // UTILITY_VISION_VISIONFIELDOBJECT_H
