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

            for(auto it = goalSegments.first; it != goalSegments.second; ++it) {

                auto& elem = it->second;
                arma::ivec2 midpoint = elem.midpoint;

                // Replicate our midpoint for each of the points
                arma::imat points = arma::repmat(midpoint, 1, 6);

                // Our even rows are moved to the left
                points(arma::uvec({ 0 }), arma::uvec({ 0, 2, 4 })) -= lround(double(elem.length) * GOAL_EXTENSION_SCALE);

                // Our odd rows are moved to the right
                points(arma::uvec({ 0 }), arma::uvec({ 1, 3, 5 })) += lround(double(elem.length) * GOAL_EXTENSION_SCALE);

                // Our top rows are moved up 1/3 of the distance
                points(arma::uvec({ 1 }), arma::uvec({ 0, 1 })) -= lround(double(GOAL_LINE_SPACING / 3));

                // Our bottom rows are moved down 1/3 of the distance
                points(arma::uvec({ 1 }), arma::uvec({ 4, 5 })) += lround(double(GOAL_LINE_SPACING / 3));

                // Our lhs must be at least 0
                points(arma::uvec({ 0 }), arma::find(points.row(0) < 0)).fill(0);

                // Our rhs must be at most the image width
                points(arma::uvec({ 0 }), arma::find(points.row(0) > int(image.width() - 1))).fill(int(image.width() - 1));

                // TODO only do the extra segments if they are above, and do not intersect the visual horizon

                if(points(1, 0) >= 0 && points(1, 0) < int(image.height())) {

                    auto segments = quex->classify(image, lut, points.col(0), points.col(1));
                    newSegments.insert(newSegments.begin(), segments.begin(), segments.end());
                }
                if(points(1, 2) >= 0 && points(1, 2) < int(image.height())) {

                    auto segments = quex->classify(image, lut, points.col(2), points.col(3));
                    newSegments.insert(newSegments.begin(), segments.begin(), segments.end());
                }
                if(points(1, 4) >= 0 && points(1, 4) < int(image.height())) {

                    auto segments = quex->classify(image, lut, points.col(4), points.col(5));
                    newSegments.insert(newSegments.begin(), segments.begin(), segments.end());
                }

            }

            insertSegments(classifiedImage, newSegments, false);

        }

    }  // vision
}  // modules