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

#ifndef MESSAGES_VISION_CLASSIFIEDIMAGE_H
#define MESSAGES_VISION_CLASSIFIEDIMAGE_H

#include <map>
#include <armadillo>

namespace messages {
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

            // Our horizon as a gradient/intercept pair
            arma::vec2 horizon;

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
                        return k > v[0];
                    }

                    bool operator()(const arma::ivec& v, const int& k) {
                        return v[0] > k;
                    }
                } equalRangeComp;

                auto points = std::equal_range(visualHorizon.rbegin(), visualHorizon.rend(), x, equalRangeComp);

                int x1 = points.second->at(0);
                int y1 = points.second->at(1);
                int x2 = points.first->at(0);
                int y2 = points.first->at(1);


                return int(double(y2 - y1) / double(x2 - x1) * double(x - x1) + y1);
            }


        };

    }  // vision
}  // messages

#endif  // MESSAGES_VISION_CLASSIFIEDIMAGE_H
