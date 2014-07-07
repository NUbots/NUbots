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

#include "LUTClassifier.h"
#include "QuexClassifier.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        void LUTClassifier::findVisualHorizon(const Image& image, const LookUpTable& lut, ClassifiedImage<ObjectClass>& classifiedImage) {

            // Get some local references to class variables to make text shorter
            auto& horizon = classifiedImage.horizon;
            auto& visualHorizon = classifiedImage.visualHorizon;
            auto& maxVisualHorizon = classifiedImage.maxVisualHorizon;
            auto& minVisualHorizon = classifiedImage.minVisualHorizon;

            // Cast lines to find our visual horizon
            for(uint x = 0; x < image.width(); x += VISUAL_HORIZON_SPACING) {

                // Find our point to classify from (slightly above the horizon)
                int top = std::max(int(lround(horizon.y(x)) - VISUAL_HORIZON_BUFFER), int(0));
                top = std::min(top, int(image.height() - 1));

                // Classify our segments
                auto segments = quex->classify(image, lut, { int(x), top }, { int(x), int(image.height() - 1) }, VISUAL_HORIZON_SUBSAMPLING);

                // Our default green point is the bottom of the screen
                arma::ivec2 greenPoint = { int(x), int(image.height() - 1) };

                // Loop through our segments to find our first green segment
                for (auto it = segments.begin(); it != segments.end(); ++it) {

                    // If this a valid green point update our information
                    if(it->colour == ObjectClass::FIELD && it->length >= VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE) {

                        greenPoint = it->start;

                        // We move our green point up by the scanning size if possible (assume more green horizon rather then less)
                        greenPoint[1] = std::max(int(greenPoint[1] - (VISUAL_HORIZON_SUBSAMPLING / 2)), 0);

                        // We found our green
                        break;
                    }
                }

                visualHorizon.push_back(std::move(greenPoint));

                insertSegments(classifiedImage, segments, true);
            }

            // If we don't have a line on the right of the image, make one
            if(image.width() - 1 % VISUAL_HORIZON_SPACING != 0) {

                // Our default green point is the bottom of the screen
                arma::ivec2 greenPoint = { int(image.width() - 1), int(image.height() - 1) };

                // Find our point to classify from (slightly above the horizon)
                int top = std::max(int(lround(horizon.y(image.width() - 1)) - VISUAL_HORIZON_BUFFER), int(0));
                top = std::min(top, int(image.height() - 1));

                arma::ivec2 start = { int(image.width() - 1), top };
                arma::ivec2 end = { int(image.width() - 1), int(image.height() - 1) };

                auto segments = quex->classify(image, lut, start, end, VISUAL_HORIZON_SUBSAMPLING);

                // Loop through our segments to find our first green segment
                for (auto it = segments.begin(); it != segments.end(); ++it) {

                    // If this a valid green point update our information
                    if(it->colour == ObjectClass::FIELD && it->length >= VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE) {
                        greenPoint = it->start;
                        // We found our green
                        break;
                    }
                }

                visualHorizon.push_back(std::move(greenPoint));
                insertSegments(classifiedImage, segments, true);
            }

            // Do a convex hull on the map points to build the horizon
            for(auto a = visualHorizon.begin(); a < visualHorizon.end() - 2;) {

                auto b = a + 1;
                auto c = a + 2;

                // Get the Z component of a cross product to check if it is concave
                bool concave = 0 <   (double(a->at(0)) - double(b->at(0))) * (double(c->at(1)) - double(b->at(1)))
                                   - (double(a->at(1)) - double(b->at(1))) * (double(c->at(0)) - double(b->at(0)));

                if(concave) {
                    visualHorizon.erase(b);
                    a = a == visualHorizon.begin() ? a : --a;
                }
                else {
                    ++a;
                }
            }

            // As this is a convex hull, the max visual horizon will always be at one of the edges
            maxVisualHorizon = visualHorizon.front()[1] > visualHorizon.back()[1] ? visualHorizon.begin() : visualHorizon.end() - 1;

            // As this is a convex function, we just need to progress till the next point is lower
            for(minVisualHorizon = visualHorizon.begin();
                minVisualHorizon < visualHorizon.end() - 1
                && minVisualHorizon->at(1) > (minVisualHorizon + 1)->at(1);
                ++minVisualHorizon);
        }

    }  // vision
}  // modules