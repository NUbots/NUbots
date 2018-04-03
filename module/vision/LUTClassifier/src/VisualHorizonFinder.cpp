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

#include "Classifier.h"

#include "utility/math/geometry/ParametricLine.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/geometry/Quad.h"
#include "utility/math/vision.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace vision {

    using message::input::CameraParameters;
    using message::input::Image;
    using message::vision::ClassifiedImage;
    using message::vision::LookUpTable;
    using utility::math::geometry::Line;
    using utility::math::geometry::Plane;
    using utility::math::geometry::Quad;
    using utility::math::vision::getCamFromImage;
    using utility::math::vision::getImageFromCam;
    using utility::nubugger::drawVisionLines;

    void LUTClassifier::findVisualHorizon(const Image& image,
                                          const LookUpTable& lut,
                                          ClassifiedImage& classifiedImage,
                                          const CameraParameters& cam) {

        // Get some local references to class variables to make text shorter
        Plane<3> horizon(convert<double, 3>(classifiedImage.horizon_normal));
        auto& visualHorizon = classifiedImage.visualHorizon;

        // Cast lines to find our visual horizon
        for (uint x = 0; x < image.dimensions[0]; x += VISUAL_HORIZON_SPACING) {
            // Find our point to classify from (slightly above the horizon)
            int horizon_Y = getImageFromCam(
                // Project down to horizon
                horizon.directionalProjection(getCamFromImage(arma::ivec2({int(x), 0}), cam), arma::vec({0, 0, 1})),
                cam)[1];
            // Find our point to classify from (slightly above the horizon)
            int top = std::max(int(horizon_Y - VISUAL_HORIZON_BUFFER), int(0));
            top     = std::min(top, int(image.dimensions[1] - 1));

            // Classify our segments
            auto segments = classifier->classify(
                image, lut, {int(x), top}, {int(x), int(image.dimensions[1] - 1)}, VISUAL_HORIZON_SUBSAMPLING);

            // Our default green point is the bottom of the screen
            arma::ivec2 greenPoint = {int(x), int(image.dimensions[1])};

            // Loop through our segments to find our first green segment
            for (auto it = segments.begin(); it != segments.end(); ++it) {

                // If this a valid green point update our information
                if (it->segmentClass == ClassifiedImage::SegmentClass::FIELD
                    && it->length >= VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE) {

                    greenPoint = convert<int, 2>(it->start);

                    // We move our green point up by the scanning size if possible (assume more green horizon rather
                    // then less)
                    greenPoint[1] = std::max(int(greenPoint[1] - (VISUAL_HORIZON_SUBSAMPLING / 2)), 0);

                    // We found our green
                    break;
                }
            }

            // Only put the green point in if it's on the screen
            if (greenPoint[1] < int(image.dimensions[1])) {
                visualHorizon.push_back(std::move(convert<int, 2>(greenPoint)));
            }

            insertSegments(classifiedImage, segments, true);
        }

        // If we don't have a line on the right of the image, make one
        if (image.dimensions[0] - 1 % VISUAL_HORIZON_SPACING != 0) {

            // Find our point to classify from (slightly above the horizon)
            int horizon_Y = getImageFromCam(
                // Project down to horizon
                horizon.directionalProjection(getCamFromImage(arma::ivec2({int(image.dimensions[0] - 1), 0}), cam),
                                              arma::vec({0, 0, 1})),
                cam)[1];
            int top = std::max(int(horizon_Y - VISUAL_HORIZON_BUFFER), int(0));
            top     = std::min(top, int(image.dimensions[1] - 1));

            arma::ivec2 start = {int(image.dimensions[0] - 1), top};
            arma::ivec2 end   = {int(image.dimensions[0] - 1), int(image.dimensions[1] - 1)};

            // Classify our segments
            auto segments = classifier->classify(image, lut, start, end, VISUAL_HORIZON_SUBSAMPLING);

            // Our default green point is the bottom of the screen
            arma::ivec2 greenPoint = {int(image.dimensions[0] - 1), int(image.dimensions[1])};

            // Loop through our segments to find our first green segment
            for (auto it = segments.begin(); it != segments.end(); ++it) {

                // If this a valid green point update our information
                if (it->segmentClass == ClassifiedImage::SegmentClass::FIELD
                    && it->length >= VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE) {
                    greenPoint = convert<int, 2>(it->start);
                    // We found our green
                    break;
                }
            }

            // Only put the green point in if it's on the screen
            if (greenPoint[1] < int(image.dimensions[1])) {
                visualHorizon.push_back(std::move(convert<int, 2>(greenPoint)));
            }

            insertSegments(classifiedImage, segments, true);
        }

        // Do a convex hull on the map points to build the horizon
        for (auto a = visualHorizon.begin(); a + 2 < visualHorizon.end();) {

            auto b = a + 1;
            auto c = a + 2;

            // Get the Z component of a cross product to check if it is concave
            bool concave = 0 < (double(a->x()) - double(b->x())) * (double(c->y()) - double(b->y()))
                                   - (double(a->y()) - double(b->y())) * (double(c->x()) - double(b->x()));

            if (concave) {
                visualHorizon.erase(b);
                a = a == visualHorizon.begin() ? a : --a;
            }
            else {
                ++a;
            }
        }

        // If we don't have any points add two
        if (visualHorizon.empty()) {
            visualHorizon.push_back(Eigen::Vector2i({0, int(image.dimensions[1] - 1)}));
            visualHorizon.push_back(Eigen::Vector2i({int(image.dimensions[0] - 1), int(image.dimensions[1] - 1)}));
        }

        else {
            // Now we need to apply our hull to the edges
            if (visualHorizon.front()[0] != 0) {
                // Our first point
                auto& a = visualHorizon.front();

                // Insert this new point at the front
                Eigen::Vector2i p(std::max(0, int(a[0] - VISUAL_HORIZON_SPACING)), int(image.dimensions[1] - 1));
                visualHorizon.insert(visualHorizon.begin(), p);

                // If this new point wasn't at 0, then add a new one there too
                if (p[0] > 0) {
                    visualHorizon.insert(visualHorizon.begin(), Eigen::Vector2i(0, int(image.dimensions[1] - 1)));
                }
            }

            if (visualHorizon.back()[0] != int(image.dimensions[0]) - 1) {
                // Our last point
                auto& a = visualHorizon.back();

                // Insert this new point at the end
                Eigen::Vector2i p(std::min(int(image.dimensions[0]) - 1, int(a[0] + VISUAL_HORIZON_SPACING)),
                                  int(image.dimensions[1]) - 1);
                visualHorizon.insert(visualHorizon.end(), p);

                // If this new point wasn't at the end, then add a new one there to
                if (p[0] < int(image.dimensions[0] - 1)) {
                    visualHorizon.insert(visualHorizon.end(),
                                         Eigen::Vector2i(int(image.dimensions[0]) - 1, int(image.dimensions[1]) - 1));
                }
            }
        }
    }

}  // namespace vision
}  // namespace module
