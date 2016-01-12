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

#include "utility/math/geometry/Line.h"

namespace module {
    namespace vision {

        using message::input::Image;
        using message::input::Sensors;
        using message::vision::LookUpTable;
        using message::vision::ObjectClass;
        using message::vision::ClassifiedImage;

        using utility::math::geometry::Line;

        void LUTClassifier::findGoals(const Image& image, const LookUpTable& lut, ClassifiedImage<ObjectClass>& classifiedImage) {

            /*
               Here we cast classification lines to attempt to locate the general area of the goals.
               We cast lines only above the visual horizon (with some buffer) so that we do not over.
               classify the mostly empty green below.
             */

            // Cast lines upward to find the goals starting at the lowest point of the visual horizon
            for(int y = 0; y < classifiedImage.maxVisualHorizon->at(1); y += GOAL_LINE_SPACING) {

                arma::ivec2 start = { 0, y };
                arma::ivec2 end = { int(image.width - 1), y };

                // Insert our segments
                auto segments = quex->classify(image, lut, start, end, GOAL_SUBSAMPLING);
                insertSegments(classifiedImage, segments, false);
            }
        }

    }  // vision
}  // modules