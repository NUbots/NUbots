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

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "messages/vision/ClassifiedImage.h"
#include "../VisionKinematics.h"
#include "../VisionFieldObject.h"

namespace modules {
    namespace vision {

        class Obstacle : public VisionFieldObject {
        public:
            Obstacle(   const VisionKinematics& visionKinematics,
                        const arma::vec2& position = arma::zeros<arma::vec>(2),
                        double width = 0,
                        double height = 0,
                        messages::vision::COLOUR_CLASS colour = messages::vision::UNKNOWN_COLOUR);

            //! @brief applies a series of checks to decide if the obstacle is valid.
            bool check() const;

            //! @brief output stream operator.
            friend std::ostream& operator<< (std::ostream& output, const Obstacle& o);

            //! @brief output stream operator for a vector of obstacles.
            friend std::ostream& operator<< (std::ostream& output, const std::vector<Obstacle>& o);

            friend class ObstacleDetector;
        private:
            /*!
              @brief calculates various positions values of the obstacle.
              @return whether the obstacle is valid.
              */
            bool calculatePositions(const VisionKinematics& visionKinematics);

            messages::vision::COLOUR_CLASS m_colour;

        //    float d2p;                      //! @variable the distance of the obstacle in cm as found by the distance to point method
            double m_arcWidth;                //! @variable the angle subtended by the obstacle (based on the screen width)
        };

    }
}
#endif // MODULES_VISION_OBSTACLE_H
