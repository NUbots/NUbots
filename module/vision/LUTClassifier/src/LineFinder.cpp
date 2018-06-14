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

        // Create visual horizon image message
        Image vHorizon;
        vHorizon.format         = utility::vision::FOURCC::GREY;
        vHorizon.dimensions.x() = image.dimensions[0];
        vHorizon.dimensions.y() = image.dimensions[1];
        vHorizon.data.resize(vHorizon.dimensions.x() * vHorizon.dimensions.y(), 0);

        // Find a bounding box for green horizon to reserve space in vHorizon image
        int greenHorzHeight = image.dimensions[1];
        for (int x = 0; x < int(image.dimensions[0]); ++x) {
            int y                                      = visualHorizonAtPoint(classifiedImage, x);
            greenHorzHeight                            = (y < greenHorzHeight) ? y : greenHorzHeight;
            vHorizon.data[y * image.dimensions[0] + x] = 255;
        }

        // Check if visual horizon could be found
        //  (if not will be a single pixel width on bottom of image)
        if (image.dimensions[1] - greenHorzHeight > 1) {
            log("New Image");
            log("\tImage dims: [", image.dimensions[0], image.dimensions[1], "]");
            log("\tGreen horizon height:", greenHorzHeight);

            Image mask;
            mask.format         = utility::vision::FOURCC::GREY;
            mask.dimensions.x() = image.dimensions[0];
            mask.dimensions.y() = image.dimensions[1];
            mask.data.resize(mask.dimensions.x() * mask.dimensions.y(), 0);

            log("\tMask data dims: [", mask.dimensions.x(), mask.dimensions.y(), "]");

            int segments = 0;

            log("\tHorizontal segments");
            // Fill mask image with field line coloured horizontal segments
            for (const auto& segment : classifiedImage.horizontalSegments) {
                // If we're within the green horizon
                if (segment.start[1] >= visualHorizonAtPoint(classifiedImage, segment.start[0])
                    && segment.end[1] >= visualHorizonAtPoint(classifiedImage, segment.end[0])) {
                    log("\t\tWithin green horz", segment.segmentClass);
                    // If the segment is of line type
                    if (segment.segmentClass == ClassifiedImage::SegmentClass::FIELD) {
                        log("\t\tAdding Segment");
                        log("\t\t\tStart: [", segment.start[0], segment.start[1], "]");
                        log("\t\t\tEnd: [", segment.end[0], segment.end[1], "]");
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
                        if (minX != maxX) {
                            segments++;
                            for (auto& x = minX; x <= maxX; ++x) {
                                mask.data[(int(lround(l.y(x)))) * mask.dimensions.x() + x] = 255;
                            }
                        }
                    }
                }
            }

            log("\tVertical segments");
            // Fill mask image with field line coloured vertical segments
            for (const auto& segment : classifiedImage.verticalSegments) {
                // If we're within the green horizon
                if (segment.start[1] >= visualHorizonAtPoint(classifiedImage, segment.start[0])
                    && segment.end[1] >= visualHorizonAtPoint(classifiedImage, segment.end[0])) {
                    log("\t\tWithin green horz", segment.segmentClass);
                    // If the segment is of line type
                    if (segment.segmentClass == ClassifiedImage::SegmentClass::FIELD) {
                        log("\t\tAdding Segment");
                        log("\t\t\tStart: [", segment.start[0], segment.start[1], "]");
                        log("\t\t\tEnd: [", segment.end[0], segment.end[1], "]");
                        // Add segment to mask image
                        // Create line for segment
                        utility::math::geometry::Line l({double(segment.start[0]), double(segment.start[1])},
                                                        {double(segment.end[0]), double(segment.end[1])});
                        // Get the min and max x to iterate across line
                        int minY = segment.start[1], maxY = segment.end[1];
                        if (segment.start[1] > segment.end[1]) {
                            minY = segment.end[1];
                            maxY = segment.start[1];
                        }

                        // Iterate through line and add each pixel
                        if (minY != maxY) {
                            segments++;
                            for (auto& y = minY; y <= maxY; ++y) {
                                mask.data[int(lround(l.x(y))) * mask.dimensions.y() + y] = 255;
                            }
                        }
                    }
                }
            }

            if (segments > 0) {
                log("Saving image");
                // DEBUG: Save the image
                utility::vision::saveImage("test.ppm", mask);
            }

            utility::vision::saveImage("vHorz.ppm", vHorizon);

            // else {
            //     log<NUClear::WARN>("Could not construct visual horizon");
        }
    }  // namespace vision
}  // namespace vision
}  // namespace module
