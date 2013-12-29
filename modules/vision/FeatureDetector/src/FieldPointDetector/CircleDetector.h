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

#ifndef MODULES_VISION_CIRCLEDETECTOR_H
#define MODULES_VISION_CIRCLEDETECTOR_H

#include <vector>

#include "../NUPoint.h"
#include "../VisionKinematics.h"
#include "../RANSAC/RANSAC.h"
#include "../RANSAC/RANSACCircle.h"

#include "CentreCircle.h"

namespace modules {
    namespace vision {

        class CircleDetector {
        public:
            CircleDetector();

            void setParameters(double TOLERANCE_,
                               unsigned int MINIMUM_POINTS_,
                               unsigned int ITERATIONS_PER_FITTING_,
                               double CONSENSUS_MARGIN_,
                               unsigned int MAX_FITTINGS_,
                               float CENTRE_CIRCLE_RADIUS_, 
                               const VisionKinematics& transformer);
            
            virtual bool run(std::vector<NUPoint>& points, CentreCircle& result);

        private:
            double TOLERANCE;
            unsigned int MINIMUM_POINTS;
            unsigned int ITERATIONS_PER_FITTING;
            double CONSENSUS_MARGIN;
            unsigned int MAX_FITTINGS;
            float CENTRE_CIRCLE_RADIUS;

            VisionKinematics m_transformer;
        };

    }
}

#endif // MODULES_VISION_CIRCLEDETECTOR_H
