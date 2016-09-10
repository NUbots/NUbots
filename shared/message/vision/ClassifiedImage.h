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

#ifndef MESSAGE_VISION_CLASSIFIEDIMAGE_H
#define MESSAGE_VISION_CLASSIFIEDIMAGE_H

#include <map>
#include <armadillo>

#include "message/input/Sensors.h"
#include "message/input/Image.h"
#include "utility/math/geometry/Line.h"

namespace message {
    namespace vision {

        enum class ObjectClass {
            UNKNOWN = 0,
            FIELD = 1,
            BALL = 2,
            GOAL = 3,
            LINE = 4,
            CYAN_TEAM = 5,
            MAGENTA_TEAM = 6,
        };

        /**
         * @brief Holds the transitions from a classifeid image
         *
         * @author Trent Houliston
         *
         * @tparam TClass the object that dividess different classes in the image.
         */
        template <typename TClass>
        struct ClassifiedImage {

            struct Segment {

                TClass colour;

                uint length;
                uint subsample;

                arma::ivec2 start;
                arma::ivec2 end;
                arma::ivec2 midpoint;

                Segment* previous;
                Segment* next;
            };

            // The sensor frame that happened with this image
            std::shared_ptr<const message::input::Sensors> sensors;

            // The image that was used to create this classified image
            std::shared_ptr<const message::input::Image> image;

            // Our images dimensions
            arma::uvec2 dimensions;

            // Points that are on the edge of the ball
            std::array<std::vector<arma::ivec2>, 3> ballSeedPoints;

            // Points that could make up the ball
            std::vector<arma::ivec2> ballPoints;

            // Our horizon
            utility::math::geometry::Line horizon;

            // The points of the visual horizon
            std::vector<arma::ivec2> visualHorizon;
            std::vector<arma::ivec2>::iterator maxVisualHorizon;
            std::vector<arma::ivec2>::iterator minVisualHorizon;

            // Our segments, split into vertical and horizontal components
            std::multimap<TClass, Segment> horizontalSegments;
            std::multimap<TClass, Segment> verticalSegments;

            int visualHorizonAtPoint(int x) const {

                struct {
                    bool operator()(const int& k, const arma::ivec& v) {
                        return k < v[0];
                    }

                    bool operator()(const arma::ivec& v, const int& k) {
                        return v[0] < k;
                    }
                } comparator;

                // Find the point such that pt1 < x < pt2

                auto p2 = std::upper_bound(visualHorizon.begin(), visualHorizon.end(), x, comparator);
                p2 -= p2 == visualHorizon.end() ? 1 : 0;
                auto p1 = p2 - 1;

                utility::math::geometry::Line l({ double(p1->at(0)), double(p1->at(1))}, {double(p2->at(0)), double(p2->at(1))});

                return int(lround(l.y(x)));
            }

        };

    }  // vision
}  // message

#endif  // MESSAGE_VISION_CLASSIFIEDIMAGE_H
