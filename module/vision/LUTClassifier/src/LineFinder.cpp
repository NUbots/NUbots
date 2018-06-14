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

#include "LUTClassifier.h"
#include "message/input/CameraParameters.h"
#include "utility/math/geometry/Line.h"
#include "utility/math/vision.h"
#include "utility/vision/ClassifiedImage.h"
#include "utility/vision/Vision.h"

namespace module {
namespace vision {

    using message::input::Image;
    using message::vision::ClassifiedImage;
    using message::vision::LookUpTable;

    using utility::math::geometry::Line;
    using utility::math::vision::getGroundPointFromScreen;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::projectWorldPointToScreen;
    using utility::math::vision::screenToImage;

    using utility::vision::visualHorizonAtPoint;

    using message::input::CameraParameters;

    void LUTClassifier::findLines(const Image& image, ClassifiedImage& classifiedImage) {
        // TODO: Possibly remove fitted ball classification models

        // Find a bounding box for green horizon to reserve space in mask image
        int greenHorzHeight = image.dimensions[1];
        for (int x = 0; x < int(image.dimensions[0]); ++x) {
            int y           = visualHorizonAtPoint(classifiedImage, x);
            greenHorzHeight = (y < greenHorzHeight) ? y : greenHorzHeight;
        }

        // Check if visual horizon could be found
        //  (if not will be a single pixel width on bottom of image)
        if (image.dimensions[1] - greenHorzHeight > 1) {
            log("Image dims:", image.dimensions);
            log("Green horizon height:", greenHorzHeight);

            // Create mask image message
            Image mask;
            mask.format         = utility::vision::FOURCC::RGB3;
            mask.dimensions.x() = image.dimensions[0];
            mask.dimensions.y() = image.dimensions[1] - greenHorzHeight;
            mask.data.resize(mask.dimensions.x() * (mask.dimensions.y() - greenHorzHeight), 0);

            log("Mask data dims:", mask.dimensions.x(), mask.dimensions.y() - greenHorzHeight);

            // Fill mask image with field line coloured segments
            for (const auto& segment : classifiedImage.horizontalSegments) {
                // log("\n\tSegment:", "\nStart:\n", segment.start, "\nEnd:\n", segment.end);
                // If we're within the green horizon
                if (segment.start[1] >= visualHorizonAtPoint(classifiedImage, segment.start[0])
                    && segment.end[1] >= visualHorizonAtPoint(classifiedImage, segment.end[0])) {
                    log("Within green horz", segment.segmentClass);
                    // If the segment is of line type
                    if (segment.segmentClass == ClassifiedImage::SegmentClass::LINE) {
                        log("Adding segment");
                        // Add segment to mask image
                        // Create line for segment
                        utility::math::geometry::Line l({double(segment.start[0]), double(segment.start[1])},
                                                        {double(segment.end[0]), double(segment.end[1])});
                        // Get the min and max x to iterate across line
                        int minX = segment.start[0], maxX = segment.end[0];
                        if (segment.start[0] > segment.end[0]) {
                            minX = segment.end[0];
                            maxX = segment.start[0];
                        }
                        // Iterate through line and add each pixel
                        for (auto& x = minX; x <= maxX; ++x) {
                            mask.data[(int(lround(l.y(x))) - greenHorzHeight) * image.dimensions[0] + x] = 1;
                        }
                    }
                }
            }

            // DEBUG: Save the image
            utility::vision::saveImage("test.ppm", mask);
        }
        else {
            log<NUClear::WARN>("Could not construct visual horizon");
        }
    }

}  // namespace vision
}  // namespace module
