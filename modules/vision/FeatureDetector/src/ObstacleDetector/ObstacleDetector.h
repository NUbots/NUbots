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

#ifndef MODULE_VISION_OBSTACLEDETECTOR_H
#define MODULE_VISION_OBSTACLEDETECTOR_H

#include <vector>

#include "messages/vision/ClassifiedImage.h"
#include "messages/vision/VisionObjects.h"
#include "messages/vision/LookUpTable.h"
#include "messages/input/Image.h"
#include "../VisionKinematics.h"

#include "Obstacle.h"

namespace modules {
    namespace vision {

        class ObstacleDetector {
        public:
            ObstacleDetector();

            std::unique_ptr< std::vector<messages::vision::Obstacle> > run(const std::vector<arma::vec2>& greenHorizon,
                                                    const messages::vision::LookUpTable& LUT,
                                                    const messages::input::Image& img,
                                                    const std::vector<messages::vision::ColourSegment>& cyanSegments,
                                                    const std::vector<messages::vision::ColourSegment>& magentaSegments,
                                                    const VisionKinematics& visionKinematics);

            void setParameters(int MIN_DISTANCE_FROM_HORIZON_,
                                 unsigned int VERTICAL_SCANLINE_SPACING_,
                                 int MIN_CONSECUTIVE_POINTS_,
                                 int MIN_COLOUR_THRESHOLD_,
                                 int MAX_OTHER_COLOUR_THRESHOLD_,
                                 int VER_THRESHOLD_,
                                 double OBJECT_THRESHOLD_MULT_);
        private:
            void appendEdgesFromSegments(const std::vector<messages::vision::ColourSegment>& segments, std::vector<arma::vec2>& pointList);

            int MIN_DISTANCE_FROM_HORIZON;
            unsigned int VERTICAL_SCANLINE_SPACING;
            int MIN_CONSECUTIVE_POINTS;
            int MIN_COLOUR_THRESHOLD;
            int MAX_OTHER_COLOUR_THRESHOLD;
            int VER_THRESHOLD;
            double OBJECT_THRESHOLD_MULT;

            std::unique_ptr< std::vector<messages::vision::Obstacle> > createObstacleMessage(std::vector<Obstacle> obstacles);
        };

    }
}

#endif // MODULE_VISION_OBSTACLEDETECTOR_H
