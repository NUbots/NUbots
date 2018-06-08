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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_CLASSIFIER_H
#define MODULES_VISION_CLASSIFIER_H

#include <vector>

#include <armadillo>

#include "message/input/Image.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/LookUpTable.h"

#include "utility/vision/Vision.h"

namespace module {
namespace vision {
    class Classifier {
    private:
        message::vision::ClassifiedImage::SegmentClass classifySegment(const utility::vision::Colour& colour) const;

    public:
        Classifier() {}

        std::vector<message::vision::ClassifiedImage::Segment> classify(const message::input::Image& image,
                                                                        const message::vision::LookUpTable& lut,
                                                                        const arma::ivec2& start,
                                                                        const arma::ivec2& end,
                                                                        const uint& stratification = 1);
    };
}  // namespace vision
}  // namespace module

#endif
