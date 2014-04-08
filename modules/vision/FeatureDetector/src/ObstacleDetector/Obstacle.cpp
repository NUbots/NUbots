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

#include "Obstacle.h"

namespace modules {
    namespace vision {

        using messages::vision::COLOUR_CLASS;

        Obstacle::Obstacle(const VisionKinematics& visionKinematics, const arma::vec2& position, double width, double height, COLOUR_CLASS colour){
            m_id = OBSTACLE;
            m_location.screenCartesian = position;
            m_sizeOnScreen = arma::vec2({width, height});
            m_colour = colour;

            // CALCULATE DISTANCE AND BEARING VALS
            valid = (calculatePositions(visionKinematics) && check());
        }



        bool Obstacle::check() const {
            //! TODO: Do a check based on width and d2p consistency
        //    if (!distance_valid) {
        //        return false;
        //    }

            // All checks passed.
            return true;
        }

        bool Obstacle::calculatePositions(const VisionKinematics& visionKinematics) {
            // To the bottom of the Goal Post..
            visionKinematics.calculateRepresentationsFromPixelLocation(m_location);

            // Find arc width.
            NUPoint gp1, gp2;
            arma::vec2 screenSize;
            screenSize << m_sizeOnScreen[0] << 0;

            gp1.screenCartesian = m_location.screenCartesian - screenSize;
            gp2.screenCartesian = m_location.screenCartesian + screenSize;

            visionKinematics.calculateRepresentationsFromPixelLocation(gp1);
            visionKinematics.calculateRepresentationsFromPixelLocation(gp2);

            m_arcWidth = std::abs(gp1.screenAngular[0] - gp2.screenAngular[0]);

            m_sphericalError = visionKinematics.calculateSphericalError(m_location, DISTANCE_METHOD::D2P, 0);

            return (m_location.neckRelativeRadial[0] > 0);
        }

        /*! @brief Stream insertion operator for a single ColourSegment.
         *      The segment is terminated by a newline.
         */
        std::ostream& operator<< (std::ostream& output, const Obstacle& obstacle) {
            output << "Obstacle" << std::endl;
            output << "\tpixelloc: " << obstacle.m_location.screenCartesian << std::endl;
            output << "\tangularloc: " << obstacle.m_location.screenAngular << std::endl;
            output << "\trelative field coords: " << obstacle.m_location.neckRelativeRadial << std::endl;
            output << "\tSize on screen: " << obstacle.m_sizeOnScreen << std::endl;
            switch(obstacle.m_colour){
                case COLOUR_CLASS::UNKNOWN_COLOUR:
                    output << "\tRobot team colour: UNKNOWN " << std::endl;
                    break;
                case COLOUR_CLASS::TEAM_MAGENTA_COLOUR:
                    output << "\tRobot team colour: TEAM_MAGENTA " << std::endl;
                    break;
                case COLOUR_CLASS::TEAM_CYAN_COLOUR:
                    output << "\tRobot team colour: TEAM_CYAN " << std::endl;
                    break;
                default:
                    output << "\tRobot team colour: Not robot colour type!!!" << std::endl;
                    break;
            }
            output << "\tSpherical error: " << obstacle.m_sphericalError << std::endl;


            return output;
        }

        /*! @brief Stream insertion operator for a vector of ColourSegments.
         *      Each segment is terminated by a newline.
         *  @relates ColourSegment
         */
        std::ostream& operator<< (std::ostream& output, const std::vector<Obstacle>& obstacles) {
            for (const auto& obstacle : obstacles) {
                output << obstacle << std::endl;
            }
            return output;
        }

    }
}
