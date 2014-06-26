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
        using messages::input::Sensors;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        void LUTClassifier::findVisualHorizon(const Image& image, const LookUpTable& lut, const Sensors& sensors, ClassifiedImage<ObjectClass>& classifiedImage) {

            auto& horizon = classifiedImage.horizon;

            std::vector<arma::uvec2> horizonPoints;

            // Cast lines to find our visual horizon
            for(uint i = 0; i < image.width(); i += VISUAL_HORIZON_SPACING) {

                // Find our point to classify from (slightly above the horizon)
                uint top = std::max(int(i * horizon[0] + horizon[1] - VISUAL_HORIZON_BUFFER), int(0));
                top = std::min(top, image.height() - 1);

                // Classify our segments
                auto segments = quex->classify(image, lut, { i, top }, { i, image.height() - 1 }, VISUAL_HORIZON_SUBSAMPLING);

                // Our default green point is the bottom of the screen
                arma::uvec2 greenPoint = { i, image.height() - 1 };

                // Loop through our segments to find our first green segment
                for (auto it = segments.begin(); it != segments.end(); ++it) {

                    // If this a valid green point update our information
                    if(it->colour == ObjectClass::FIELD && it->length >= MINIMUM_VISUAL_HORIZON_SEGMENT_SIZE) {

                        greenPoint = it->start;

                        // We move our green point up by the scanning size if possible (assume more green horizon rather then less)
                        greenPoint[1] = std::max(int(greenPoint[1] - (VISUAL_HORIZON_SUBSAMPLING / 2)), 0);

                        // We found our green
                        break;
                    }
                }

                horizonPoints.push_back(std::move(greenPoint));

                insertSegments(classifiedImage, segments, true);
            }

            // If we don't have a line on the right of the image, make one
            if(image.width() - 1 % VISUAL_HORIZON_SPACING != 0) {

                // Our default green point is the bottom of the screen
                arma::uvec2 greenPoint = { image.width() - 1, image.height() - 1 };

                // Find our point to classify from (slightly above the horizon)
                uint top = std::max(int((image.width() - 1) * horizon[0] + horizon[1] - VISUAL_HORIZON_BUFFER), int(0));
                top = std::min(top, image.height() - 1);

                arma::uvec2 start = { image.width() - 1, top };
                arma::uvec2 end = { image.width() - 1, image.height() - 1 };

                auto segments = quex->classify(image, lut, start, end, VISUAL_HORIZON_SUBSAMPLING);

                // Loop through our segments to find our first green segment
                for (auto it = segments.begin(); it != segments.end(); ++it) {

                    // If this a valid green point update our information
                    if(it->colour == ObjectClass::FIELD && it->length >= MINIMUM_VISUAL_HORIZON_SEGMENT_SIZE) {
                        greenPoint = it->start;
                        // We found our green
                        break;
                    }
                }

                horizonPoints.push_back(std::move(greenPoint));

                insertSegments(classifiedImage, segments, true);

            }

            // Do a convex hull on the map points to build the horizon
            for(auto a = horizonPoints.begin(); a < horizonPoints.end() - 2;) {

                auto b = a + 1;
                auto c = a + 2;

                // Get the Z component of a cross product to check if it is concave
                bool concave = 0 <   (double(a->at(0)) - double(b->at(0))) * (double(c->at(1)) - double(b->at(1)))
                                   - (double(a->at(1)) - double(b->at(1))) * (double(c->at(0)) - double(b->at(0)));

                if(concave) {
                    horizonPoints.erase(b);
                    a = a == horizonPoints.begin() ? a : --a;
                }
                else {
                    ++a;
                }
            }

            uint maxVisualHorizon = 0;
            uint minVisualHorizon = image.height();

            for(uint i = 0; i < horizonPoints.size() - 1; ++i) {
                const auto& p1 = horizonPoints[i];
                const auto& p2 = horizonPoints[i + 1];

                maxVisualHorizon = std::max({ maxVisualHorizon, uint(p1[1]), uint(p2[1]) });
                minVisualHorizon = std::min({ minVisualHorizon, uint(p1[1]), uint(p2[1]) });

                double m = (double(p2[1]) - double(p1[1])) / (double(p2[0]) - double(p1[0]));
                double b = - m * double(p1[0]) + double(p1[1]);

                classifiedImage.visualHorizon.push_back({ double(p1[0]), m, b });
            }
        }

    }  // vision
}  // modules