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

#include <cmath>

#include "message/input/CameraParameters.h"
#include "utility/math/geometry/Line.h"
#include "utility/math/geometry/ParametricLine.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/vision.h"
#include "utility/vision/ClassifiedImage.h"
#include "utility/vision/Vision.h"

namespace module {
namespace vision {

    using message::input::Image;
    using message::vision::ClassifiedImage;
    using message::vision::LookUpTable;

    using utility::math::geometry::Line;
    using utility::math::geometry::Plane;
    using utility::math::vision::getCamFromImage;
    using utility::math::vision::getGroundPointFromScreen;
    using utility::math::vision::getImageFromCam;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::projectWorldPointToScreen;
    using utility::math::vision::screenToImage;

    using utility::vision::visualHorizonAtPoint;

    using message::input::CameraParameters;
    using message::input::Sensors;


    // TESTING
    //  Projects an image from radial to rectangular lens perspectives
    Image projectToCameraParams(const Image& image,
                                const CameraParameters& cam1,
                                const CameraParameters& cam2,
                                const std::string name) {

        // Create rectangular projection
        auto proj            = std::make_unique<Image>();
        proj->format         = utility::vision::FOURCC::RGB3;
        proj->dimensions.x() = image.dimensions[0];
        proj->dimensions.y() = image.dimensions[1];
        proj->data.resize(3 * image.dimensions[0] * image.dimensions[1], 0);

        // Iterate through pixels and project each one
        for (int x = 0; x < int(image.dimensions[0]); ++x) {
            for (int y = 0; y < int(image.dimensions[1]); ++y) {
                arma::ivec2 pixelMap = getImageFromCam(getCamFromImage(arma::ivec2({x, y}), cam1), cam2);
                // Bounds check new coordinates
                if (pixelMap[0] > 0 && pixelMap[0] < int(proj->dimensions.x()) && pixelMap[1] > 0
                    && pixelMap[1] < int(proj->dimensions.y())) {
                    for (int channel = 0; channel < 3; channel++) {
                        proj->data[3 * (pixelMap[1] * proj->dimensions.x() + pixelMap[0]) + channel] =
                            image.data[3 * (y * image.dimensions.x() + x) + channel];
                    }
                }
            }
        }

        // Save our image
        utility::vision::saveImage(name, *proj);
        return *proj;
    }

    Image projectToTopDown(const Image& image,
                           const CameraParameters& cam,
                           const Sensors& sensors,
                           const std::string name) {
        // Create rectangular projection
        auto topImage            = std::make_unique<Image>();
        topImage->format         = utility::vision::FOURCC::RGB3;
        topImage->dimensions.x() = image.dimensions[0];
        topImage->dimensions.y() = image.dimensions[1];
        topImage->data.resize(3 * image.dimensions[0] * image.dimensions[1], 0);

        // Create plane normal to ground
        Plane<3> p(arma::vec3({0, 0, 1}));

        Eigen::Affine3d Htc(sensors.forwardKinematics[utility::input::ServoID::HEAD_PITCH]);
        // Htc(1, 3) += model->head.INTERPUPILLARY_DISTANCE * 0.5f * (i.isLeft ? 1.0f : -1.0f);
        auto Hcw             = Htc.inverse() * sensors.world;
        arma::vec3 cameraPos = convert<double, 3>(Hcw.col(3).head<3>());

        // Iterate through pixels and project each one
        for (int x = 0; x < int(image.dimensions[0]); ++x) {
            for (int y = 0; y < int(image.dimensions[1]); ++y) {

                arma::vec3 camspace = getCamFromImage(arma::ivec2({x, y}), cam);
                auto camToGround    = convert<double, 4, 4>(sensors.camToGround);
                auto groundspace    = utility::math::matrix::Transform3D(camToGround).transformVector(camspace);
                auto topInCam       = p.intersect(utility::math::geometry::ParametricLine<3>(cameraPos, groundspace));
                auto pixelMap       = getImageFromCam(topInCam, cam);

                // NUClear::log("Pixel map", pixelMap);

                // Write over pixels to new coordinates
                if (pixelMap[0] > 0 && pixelMap[0] < int(topImage->dimensions.x()) && pixelMap[1] > 0
                    && pixelMap[1] < int(topImage->dimensions.y())) {
                    for (int channel = 0; channel < 3; channel++) {
                        topImage->data[3 * (pixelMap[1] * topImage->dimensions.x() + pixelMap[0]) + channel] =
                            image.data[3 * (y * image.dimensions.x() + x) + channel];
                    }
                }
            }
        }

        // Save our image
        NUClear::log("Saving", name);
        utility::vision::saveImage(name, *topImage);
        NUClear::log("Saved", name);

        return *topImage;
    }

    void LUTClassifier::findLines(const Image& image, ClassifiedImage& classifiedImage, const Sensors& sensors) {
        // Create spherical cam for projection
        auto sphericalCam                    = std::make_unique<CameraParameters>();
        sphericalCam->imageSizePixels        = {image.dimensions[0], image.dimensions[1]};
        sphericalCam->FOV                    = {M_PI, M_PI};
        sphericalCam->centreOffset           = {0, 0};
        sphericalCam->lens                   = CameraParameters::LensType::RADIAL;
        sphericalCam->radial.radiansPerPixel = 0.0026768;

        // Create rectangular cam for projection
        auto rectCam                       = std::make_unique<CameraParameters>();
        rectCam->imageSizePixels           = {image.dimensions[0], image.dimensions[1]};
        rectCam->FOV                       = {140 * M_PI / 180., 140 * M_PI / 180.};
        rectCam->centreOffset              = {0, 0};
        rectCam->lens                      = CameraParameters::LensType::PINHOLE;
        rectCam->pinhole.distortionFactor  = 0;
        arma::vec2 tanHalfFOV              = {std::tan(rectCam->FOV[0] * 0.5), std::tan(rectCam->FOV[0] * 0.5)};
        arma::vec2 imageCentre             = {image.dimensions[0] * 0.5, image.dimensions[1] * 0.5};
        rectCam->pinhole.focalLengthPixels = imageCentre[0] / tanHalfFOV[0];
        rectCam->pinhole.pixelsToTanThetaFactor << (tanHalfFOV[0] / imageCentre[0]), tanHalfFOV[1] / imageCentre[1];

        auto proj = projectToCameraParams(image, *sphericalCam, *rectCam, "projection.ppm");

        auto top = projectToTopDown(image, *rectCam, sensors, "top_down.ppm");

        // log("Finished");
        // // Create visual horizon image message
        // Image vHorizon;
        // vHorizon.format         = utility::vision::FOURCC::GREY;
        // vHorizon.dimensions.x() = image.dimensions[0];
        // vHorizon.dimensions.y() = image.dimensions[1];
        // vHorizon.data.resize(vHorizon.dimensions.x() * vHorizon.dimensions.y(), 0);

        // // Find a bounding box for green horizon to reserve space in vHorizon image
        // int greenHorzHeight = image.dimensions[1];
        // for (int x = 0; x < int(image.dimensions[0]); ++x) {
        //     int y                                      = visualHorizonAtPoint(classifiedImage, x);
        //     greenHorzHeight                            = (y < greenHorzHeight) ? y : greenHorzHeight;
        //     vHorizon.data[y * image.dimensions[0] + x] = 255;
        // }

        // log("Saving visual horizon");
        // utility::vision::saveImage("vHorz.ppm", vHorizon);
        // log("Saved visual horizon");


        // // Check if visual horizon could be found
        // //  (if not will be a single pixel width on bottom of image)
        // if (image.dimensions[1] - greenHorzHeight > 1) {
        //     log("New Image");
        //     log("\tImage dims: [", image.dimensions[0], image.dimensions[1], "]");
        //     log("\tGreen horizon height:", greenHorzHeight);

        //     Image mask;
        //     mask.format         = image.format;
        //     mask.dimensions.x() = image.dimensions[0];
        //     mask.dimensions.y() = image.dimensions[1];
        //     mask.data.resize(mask.dimensions.x() * mask.dimensions.y(), 0);

        //     // log("\tMask data dims: [", mask.dimensions.x(), mask.dimensions.y(), "]");

        //     int segments = 0;

        //     // log("\tHorizontal segments");
        //     // Fill mask image with field line coloured horizontal segments
        //     for (const auto& segment : classifiedImage.horizontalSegments) {
        //         log(segment.start, segment.end);
        //         // If we're within the green horizon
        //         if (segment.start[1] >= 0 && segment.end[1] >= 0) {
        //             // If the segment is of line type
        //             if (segment.segmentClass == ClassifiedImage::SegmentClass::GOAL
        //                 || segment.segmentClass == ClassifiedImage::SegmentClass::LINE) {
        //                 // log("\t\tAdding Segment");
        //                 // log("\t\t\tStart: [", segment.start[0], segment.start[1], "]");
        //                 // log("\t\t\tEnd: [", segment.end[0], segment.end[1], "]");
        //                 // Add segment to mask image
        //                 // Create line for segment
        //                 utility::math::geometry::Line l({double(segment.start[0]), double(segment.start[1])},
        //                                                 {double(segment.end[0]), double(segment.end[1])});

        //                 // Get the min and max x to iterate across line
        //                 int minX = segment.start[0], maxX = segment.end[0];
        //                 if (segment.start[0] > segment.end[0]) {
        //                     minX = segment.end[0];
        //                     maxX = segment.start[0];
        //                 }

        //                 // Iterate through line and add each pixel
        //                 if (minX != maxX) {
        //                     segments++;
        //                     for (auto& x = minX; x <= maxX; ++x) {
        //                         int y = int(lround(l.y(x)));
        //                         if (y > 0 && y < mask.dimensions.y() && x > 0 && x < mask.dimensions.x()) {
        //                             mask.data[y * mask.dimensions.x() + x] = 255;
        //                         }
        //                     }
        //                 }
        //             }
        //         }
        //     }

        //     if (segments > 0 && (int(mask.data[0]) >= 0 || int(mask.data[0] <= 255))) {
        //         // DEBUG: Save the image
        //         log("Saving mask");
        //         utility::vision::saveImage("test.ppm", mask);
        //         log("Saved mask");
        //     }


        // else {
        //     log<NUClear::WARN>("Could not construct visual horizon");
    }
}  // namespace vision
   // namespace module
}  // namespace module
