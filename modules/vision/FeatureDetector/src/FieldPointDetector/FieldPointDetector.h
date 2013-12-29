/*
 * This file is part of FeatureDetector.
 *
 * FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_FIELDPOINTDETECTOR_H
#define MODULES_VISION_FIELDPOINTDETECTOR_H

#include <vector>
#include <armadillo>

#include "messages/vision/ClassifiedImage.h"

#include "../VisionKinematics.h"
#include "../NUPoint.h"
#include "CircleDetector.h"
#include "LineDetector.h"
#include "CornerDetector.h"

namespace modules {
    namespace vision {

        class FieldPointDetector {
        public:
            FieldPointDetector();

            void setParameters(bool TRANSFORM_FIRST_,
                               const VisionKinematics& transformer,
                    
                               // CircleDetector.
                               double CIRCLE_DETECTOR_TOLERANCE_,
                               unsigned int MINIMUM_POINTS_,
                               unsigned int ITERATIONS_PER_FITTING_,
                               double CONSENSUS_MARGIN_,
                               unsigned int MAX_FITTINGS_,
                               float CENTRE_CIRCLE_RADIUS_, 
                    
                               // CornerDetector.
                               double CORNER_DETECTOR_TOLERANCE_);

            void run(bool findCircles,
                    bool findLines, 
                    bool findCorners, 
                    const std::vector<arma::vec2>& greenHorizon);

        private:
            LineDetector m_lineDetector;
            CircleDetector m_circleDetector;
            CornerDetector m_cornerDetector;
            
            bool TRANSFORM_FIRST;

            VisionKinematics m_transformer;

            // CircleDetector.
            double CIRCLE_DETECTOR_TOLERANCE;
            unsigned int MINIMUM_POINTS;
            unsigned int ITERATIONS_PER_FITTING;
            double CONSENSUS_MARGIN;
            unsigned int MAX_FITTINGS;
            float CENTRE_CIRCLE_RADIUS;

            // CornerDetector.
            double CORNER_DETECTOR_TOLERANCE;
        };
    }
}

#endif // MODULES_VISION_FIELDPOINTDETECTOR_H
