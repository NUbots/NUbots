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
        // Create spherical cam for projection
        CameraParameters sphericalCam;
        sphericalCam.imageSizePixels        = {320, 240};
        sphericalCam.FOV                    = {3.14, 3.14};
        sphericalCam.centreOffset           = {0, 0};
        sphericalCam.lens                   = CameraParameters::LensType::RADIAL;
        sphericalCam.radial.radiansPerPixel = 0.01;

        // Create rectangular cam for projection
        CameraParameters rectCam;
        rectCam.imageSizePixels           = {320, 240};
        rectCam.FOV                       = {1.0472, 0.785};
        rectCam.centreOffset              = {0, 0};
        rectCam.lens                      = CameraParameters::LensType::PINHOLE;
        rectCam.pinhole.distortionFactor  = 0;
        arma::vec2 tanHalfFOV             = {std::tan(rectCam.FOV[0] * 0.5), std::tan(rectCam.FOV[0] * 0.5)};
        arma::vec2 imageCentre            = {rectCam.imageSizePixels[0] * 0.5, rectCam.imageSizePixels[1] * 0.5};
        rectCam.pinhole.focalLengthPixels = imageCentre[0] / tanHalfFOV[0];
        rectCam.pinhole.pixelsToTanThetaFactor << (tanHalfFOV[0] / imageCentre[0]), tanHalfFOV[1] / imageCentre[1];

        // Create rectangular projection
        Image proj;
        proj.format         = utility::vision::FOURCC::GREY;
        proj.dimensions.x() = image.dimensions[0];
        proj.dimensions.y() = image.dimensions[1];
        proj.data.resize(proj.dimensions.x() * proj.dimensions.y(), 0);

        log("Projection");

        // Iterate through pixels and transform each one
        for (int x = 0; x < image.dimensions[0]; ++x) {
            for (int y = 0; y < image.dimensions[1]; ++y) {
                arma::vec3 camVec    = utility::math::vision::getCamFromImage(arma::ivec2({x, y}), sphericalCam);
                arma::ivec2 pixelMap = utility::math::vision::getImageFromCam(camVec, rectCam);
                // Bounds check new coordinates
                if (pixelMap[0] > 0 && pixelMap[0] < proj.dimensions.x() && pixelMap[1] > 0
                    && pixelMap[1] < proj.dimensions.y()) {
                    // log("From:", x, y, "To:", pixelMap[0], pixelMap[1]);
                    proj.data[pixelMap[1] * proj.dimensions.x() + pixelMap[0]] =
                        image.data[y * image.dimensions.x() + x];
                }
            }
        }


        utility::vision::saveImage("projection.ppm", proj);

        log("Finished");
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

        log("Saving visual horizon");
        utility::vision::saveImage("vHorz.ppm", vHorizon);
        log("Saved visual horizon");


        // Check if visual horizon could be found
        //  (if not will be a single pixel width on bottom of image)
        if (image.dimensions[1] - greenHorzHeight > 1) {
            log("New Image");
            log("\tImage dims: [", image.dimensions[0], image.dimensions[1], "]");
            log("\tGreen horizon height:", greenHorzHeight);

            Image mask;
            mask.format         = image.format;
            mask.dimensions.x() = image.dimensions[0];
            mask.dimensions.y() = image.dimensions[1];
            mask.data.resize(mask.dimensions.x() * mask.dimensions.y(), 0);

            // log("\tMask data dims: [", mask.dimensions.x(), mask.dimensions.y(), "]");

            int segments = 0;

            // log("\tHorizontal segments");
            // Fill mask image with field line coloured horizontal segments
            for (const auto& segment : classifiedImage.horizontalSegments) {
                log(segment.start, segment.end);
                // If we're within the green horizon
                if (segment.start[1] >= 0 && segment.end[1] >= 0) {
                    // If the segment is of line type
                    if (segment.segmentClass == ClassifiedImage::SegmentClass::GOAL
                        || segment.segmentClass == ClassifiedImage::SegmentClass::LINE) {
                        // log("\t\tAdding Segment");
                        // log("\t\t\tStart: [", segment.start[0], segment.start[1], "]");
                        // log("\t\t\tEnd: [", segment.end[0], segment.end[1], "]");
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
                                int y = int(lround(l.y(x)));
                                if (y > 0 && y < mask.dimensions.y() && x > 0 && x < mask.dimensions.x()) {
                                    mask.data[y * mask.dimensions.x() + x] = 255;
                                }
                            }
                        }
                    }
                }
            }

            if (segments > 0 && (int(mask.data[0]) >= 0 || int(mask.data[0] <= 255))) {
                // DEBUG: Save the image
                log("Saving mask");
                utility::vision::saveImage("test.ppm", mask);
                log("Saved mask");
            }


            // else {
            //     log<NUClear::WARN>("Could not construct visual horizon");
        }
    }  // namespace vision
}  // namespace vision
}  // namespace module
