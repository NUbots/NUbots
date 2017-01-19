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

#ifndef UTILITY_VISION_CLASSIFIEDIMAGE_H
#define UTILITY_VISION_CLASSIFIEDIMAGE_H

#include <Eigen/Core>

#include "message/vision/proto/ClassifiedImage.h"

#include "utility/math/geometry/Line.h"

namespace utility {
    namespace vision {

        int visualHorizonAtPoint(const message::vision::proto::ClassifiedImage& classifiedImage, int x) {

            struct {
                bool operator()(const int& k, const Eigen::Vector2i& v) {
                    return k < v[0];
                }

                bool operator()(const Eigen::Vector2i& v, const int& k) {
                    return v[0] < k;
                }
            } comparator;

            // Find the point such that pt1 < x < pt2
            auto p2 = std::upper_bound(classifiedImage.visualHorizon.begin(), classifiedImage.visualHorizon.end(), x, comparator);
            p2 -= p2 == classifiedImage.visualHorizon.end() ? 1 : 0;
            auto p1 = p2 - 1;

            utility::math::geometry::Line l({ double(p1->x()), double(p1->y())}, {double(p2->x()), double(p2->y())});

            return int(lround(l.y(x)));
        }

    }  // vision
}  // utility

#endif  // UTILITY_VISION_CLASSIFIEDIMAGE_H
