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

#ifndef MODULES_VISION_FEATUREDETECTOR_RANSACLINE_H
#define MODULES_VISION_FEATUREDETECTOR_RANSACLINE_H

#include "utility/math/Line.h"

#include "../NUPoint.h"
namespace modules{
    namespace vision{

        template<typename T>
        class RANSACLine : public utility::math::Line
        {
        public:
            RANSACLine() {}

            bool regenerate(const std::vector<T>& pts) {
                if(pts.size() == minPointsForFit()) {
                    setLineFromPoints(pts.at(0), pts.at(1));
                    return true;
                }
                else {
                    return false;
                }
            }

            inline size_t minPointsForFit() const {return 2;}

            double calculateError(T p) const { return getLinePointDistance(p); }
        };


        template<>
        class RANSACLine<NUPoint> : public utility::math::Line
        {
        public:
            RANSACLine() {}

            bool regenerate(const std::vector<NUPoint> &pts) {
                if(pts.size() == minPointsForFit()) {
                    setLineFromPoints(pts.at(0).groundCartesian, pts.at(1).groundCartesian);
                    return true;
                }
                else {
                    return false;
                }
            }

            inline size_t minPointsForFit() const { return 3; }

            double calculateError(NUPoint p) const { return getLinePointDistance(p.groundCartesian); }
        };


    }
}

#endif