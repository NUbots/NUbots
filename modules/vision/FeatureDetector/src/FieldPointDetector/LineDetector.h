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

#ifndef MODULES_VISION_LINEDETECTOR_H
#define MODULES_VISION_LINEDETECTOR_H

#include <vector>

#include "../LSFittedLine.h"
#include "../NUPoint.h"

#include "FieldLine.h"

namespace modules {
    namespace vision {

        class LineDetector {
        public:
            LineDetector();
            virtual ~LineDetector();

            std::vector<FieldLine> run(const std::vector<NUPoint>& points);

        protected:
            std::vector<std::pair<LSFittedLine, LSFittedLine>> mergeColinear(std::vector<std::pair<LSFittedLine, LSFittedLine>> lines,
                                                                    double angleThreshold, double distanceThreshold) const;

        };

    }
}
#endif // MODULES_VISION_LINEDETECTOR_H
