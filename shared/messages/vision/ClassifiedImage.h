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

                arma::uvec2 start;
                arma::uvec2 end;
                arma::uvec2 midpoint;

                Segment* previous;
                Segment* next;
            };

            arma::vec2 horizon;

            std::vector<arma::vec> visualHorizon;

            std::multimap<TClass, Segment> horizontalSegments;
            std::multimap<TClass, Segment> verticalSegments;
        };

    }  // vision
}  // messages

#endif  // MESSAGES_VISION_CLASSIFIEDIMAGE_H
