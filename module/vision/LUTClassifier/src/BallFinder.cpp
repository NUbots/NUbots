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

#include "utility/math/geometry/Line.h"
#include "utility/math/vision.h"

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

    void LUTClassifier::findBall(const Image& image, const LookUpTable& lut, ClassifiedImage& classifiedImage) {

        /*
            Here we cast lines to find balls.
            To do this, we cast lines seperated so that any ball will have at least 2 lines
            passing though it (possibly 3).
            This means that lines get logrithmically less dense as we decend the image as a balls
            apparent size will be larger.
            These lines are cast from slightly above the visual horizon to a point where it is needed
            (for the logrithmic grid)
         */

        auto& visualHorizon = classifiedImage.visualHorizon;

        // As this is a convex function, we just need to progress till the next point is lower
        std::vector<Eigen::Matrix<int, 2, 1, Eigen::DontAlign>>::iterator minHorizon;
        for (minHorizon = visualHorizon.begin();
             minHorizon < visualHorizon.end() - 1 && minHorizon->y() > (minHorizon + 1)->y();
             ++minHorizon)
            ;

        const auto& maxVisualHorizon =
            visualHorizon.front()[1] > visualHorizon.back()[1] ? visualHorizon.begin() : visualHorizon.end() - 1;

        arma::vec2 topY = imageToScreen(arma::ivec2({maxVisualHorizon->x(), int(maxVisualHorizon->y())}),
                                        convert(classifiedImage.dimensions));
        topY[0]         = 0;  // Choose centre of screen

        // Get the positions of the top of our green horizion, and the bottom of the screen
        arma::mat44 Hgc     = convert(classifiedImage.sensors->Hgc);
        auto xb             = getGroundPointFromScreen({0, -double(image.dimensions[1] - 1) / 2}, Hgc, image.lens);
        auto xt             = getGroundPointFromScreen(topY, Hgc, image.lens);
        double dx           = 2 * BALL_RADIUS / BALL_MINIMUM_INTERSECTIONS_COARSE;
        double cameraHeight = Hgc(2, 3);

        // This describes the direction of travel
        arma::vec3 direction = arma::normalise(xb);

        // Don't bother drawing lines if we know it's going to fail
        if (direction[0] < 0) {
            return;
        }

        // Our start and end points
        double xStart = arma::norm(xb);
        xStart += dx - fmod(xStart, dx);
        double xEnd = arma::norm(xt);

        auto movement = arma::normalise(xb) * dx;

        auto hLeft  = visualHorizon.begin();
        auto hRight = visualHorizon.end() - 1;


        // Do our inital calculation to get our first Y
        arma::vec4 worldPosition = arma::ones(4);
        worldPosition.rows(0, 2) = xStart * direction;
        auto camPoint            = projectWorldPointToScreen(worldPosition, Hgc, image.lens);
        int y                    = screenToImage(camPoint, convert(classifiedImage.dimensions))[1];

        for (double x = xStart; x < xEnd && y >= 0; x += std::max(dx, (dx * x) / (cameraHeight - dx))) {

            // Calculate our next Y
            worldPosition.rows(0, 2) = (x + std::max(dx, (dx * x) / (cameraHeight - dx))) * direction;
            camPoint                 = projectWorldPointToScreen(worldPosition, Hgc, image.lens);
            int nextY                = screenToImage(camPoint, convert(classifiedImage.dimensions))[1];

            // Work out our details
            arma::ivec2 start = {0, y};
            arma::ivec2 end   = {int(image.dimensions[0] - 1), y};
            int subsample     = std::max(1, int(lround((y - nextY) * BALL_HORIZONTAL_SUBSAMPLE_FACTOR)));

            // If our left hand side is in range, or we are over the top
            if (hLeft->y() >= y) {

                while (hLeft < minHorizon) {

                    auto p1 = hLeft;
                    auto p2 = hLeft + 1;

                    if (y <= p1->y() && y >= p2->y()) {

                        // Make a line from the two points and find our x
                        Line l({double(p1->x()), double(p1->y())}, {double(p2->x()), double(p2->y())});

                        if (l.isHorizontal()) {
                            start[0] = p2->x();
                        }
                        else {
                            start[0] = round(l.x(y));
                        }

                        break;
                    }
                    // Try our previous point
                    else {
                        ++hLeft;
                    }
                }
            }

            // If our right hand side is in range and has not gone out of scope
            if (hRight->y() >= y) {

                while (hRight > minHorizon) {

                    auto p1 = hRight - 1;
                    auto p2 = hRight;

                    if (y >= p1->y() && y <= p2->y()) {

                        // Make a line from the two points and find our x
                        Line l({double(p1->x()), double(p1->y())}, {double(p2->x()), double(p2->y())});

                        if (l.isHorizontal()) {
                            end[0] = p1->x();
                        }
                        else {
                            end[0] = round(l.x(y));
                        }

                        break;
                    }
                    // Try our previous point
                    else {
                        --hRight;
                    }
                }
            }

            // Our Y is now our next y
            y = nextY;

            auto segments = classifier->classify(image, lut, start, end, subsample);
            insertSegments(classifiedImage, segments, false);
        }
    }

}  // namespace vision
}  // namespace module
