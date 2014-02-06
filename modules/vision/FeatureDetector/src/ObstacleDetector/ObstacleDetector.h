/**
*   @name   ObjectDetectionCH
*   @file   objectdetectionch.h
*   @brief  basic object detection by checking breaks in green horizon.
*   @author David Budden
*   @date   22/02/2012
*/

#ifndef MODULE_VISION_OBSTACLEDETECTOR_H
#define MODULE_VISION_OBSTACLEDETECTOR_H

#include <vector>

#include "messages/vision/ClassifiedImage.h"
#include "messages/vision/VisionObjects.h"
#include "utility/vision/LookUpTable.h"
#include "messages/input/Image.h"
#include "../VisionKinematics.h"

#include "Obstacle.h"

namespace modules {
    namespace vision {

        class ObstacleDetector {
        public:
            ObstacleDetector();

            std::unique_ptr< std::vector<messages::vision::Obstacle> > run(const std::vector<arma::vec2>& greenHorizon,
                                                    const utility::vision::LookUpTable& LUT,
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
