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

#ifndef MODULES_VISION_CORNERDETECTOR_H
#define MODULES_VISION_CORNERDETECTOR_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "messages/vision/VisionObjects.h"

#include "CornerPoint.h"
#include "FieldLine.h"

namespace modules {
    namespace vision {

        class CornerDetector {
        public:
            CornerDetector();

            std::vector<CornerPoint> run(const std::vector<FieldLine>& lines) const;

            void setParameters(double TOLERANCE_);

        private:
            messages::vision::CornerPoint::Type findCorner(const std::vector<NUPoint>& ep1, std::vector<NUPoint>& ep2, const NUPoint& intersection, double tolerance) const;

            double TOLERANCE;
        };

    }
}

#endif // MODULES_VISION_CORNERDETECTOR_H
