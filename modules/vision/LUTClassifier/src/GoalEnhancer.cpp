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

        void LUTClassifier::enhanceGoals(const Image& image, const LookUpTable& lut, ClassifiedImage<ObjectClass>& classifiedImage) {

            /*
                Here we improve the classification of goals.
                We do this by taking our course classification of the whole image
                and generating new segments where yellow was detected.
                We first generate segments above and below that are 2x the width of the segment
             */

            std::vector<ClassifiedImage<ObjectClass>::Segment> newSegments;
            auto goalSegments = classifiedImage.horizontalSegments.equal_range(ObjectClass::GOAL);

            // Draw 2n + 1 lines each n/(2n - 1) apart
            for(auto it = goalSegments.first; it != goalSegments.second; ++it) {

                auto& elem = it->second;

                // We need 2n + 1 points, each with a start and end based on the midpoint
                arma::mat points = arma::repmat(arma::vec({ double(elem.midpoint[0]), double(elem.midpoint[1]) }), 2, 2 * GOAL_LINE_DENSITY + 1);

                // Move our X coordinates left and right
                points.each_col() += arma::vec({ -double(elem.length) * GOAL_EXTENSION_SCALE, 0, double(elem.length) * GOAL_EXTENSION_SCALE, 0 });

                // Generate all our Y values for our points
                points.row(1) += GOAL_LINE_SPACING * ((arma::linspace(-GOAL_LINE_DENSITY, GOAL_LINE_DENSITY, 2 * GOAL_LINE_DENSITY + 1).t() * GOAL_LINE_DENSITY) / (2 * GOAL_LINE_DENSITY + 1));
                // Copy over to the end
                points.row(3) = points.row(1);

                // Our lhs must be at least 0
                points(arma::uvec({ 0 }), arma::find(points.row(0) < 0)).fill(0);

                // Our rhs must be at most the image width
                points(arma::uvec({ 2 }), arma::find(points.row(2) > double(image.width - 1))).fill(double(image.width - 1));

                // Classify each of our points
                for(uint i = 0; i < points.n_cols; ++i) {

                    auto element = points.col(i);

                    // Check our Y is within the bounds (no need to check the end since they are the same)
                    if(element(1) >= 0 && element(1) < int(image.height)) {
                        auto segments = quex->classify(image, lut, { int(element(0)), int(element(1)) }, { int(element(2)), int(element(3)) });
                        newSegments.insert(newSegments.begin(), segments.begin(), segments.end());
                    }
                }
            }

            insertSegments(classifiedImage, newSegments, false);

        }

    }  // vision
}  // modules