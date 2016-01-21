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

#include "utility/math/vision.h"

namespace module {
    namespace vision {

        using message::input::Image;
        using message::input::ServoID;
        using message::input::Sensors;
        using message::vision::LookUpTable;
        using message::vision::ObjectClass;
        using message::vision::ClassifiedImage;

        using utility::math::geometry::Line;
        using utility::math::vision::getGroundPointFromScreen;
        using utility::math::vision::projectWorldPointToScreen;
        using utility::math::vision::screenToImage;
        using utility::math::vision::imageToScreen;

        void LUTClassifier::findBall(const Image& image, const LookUpTable& lut, ClassifiedImage<ObjectClass>& classifiedImage) {

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
            auto& minHorizon = classifiedImage.minVisualHorizon;
            auto& sensors = *classifiedImage.sensors;

            arma::vec2 topY = imageToScreen(arma::ivec2({ classifiedImage.maxVisualHorizon->at(0), int(classifiedImage.maxVisualHorizon->at(1)) })
                                          , classifiedImage.dimensions);
            topY[0] = 0;    //Choose centre of screen

            // Get the positions of the top of our green horizion, and the bottom of the screen
            auto xb = getGroundPointFromScreen({ 0, -double(image.height - 1) / 2}, sensors.orientationCamToGround, FOCAL_LENGTH_PIXELS);
            auto xt = getGroundPointFromScreen(topY, sensors.orientationCamToGround, FOCAL_LENGTH_PIXELS);
            double dx = 2 * BALL_RADIUS / BALL_MINIMUM_INTERSECTIONS_COARSE;
            double cameraHeight = sensors.orientationCamToGround(2,3);

            // This describes the direction of travel
            arma::vec3 direction = arma::normalise(xb);

            // Don't bother drawing lines if we know it's going to fail
            if(direction[0] < 0) {
                return;
            }

            // Our start and end points
            double xStart = arma::norm(xb);
            xStart += dx - fmod(xStart, dx);
            double xEnd = arma::norm(xt);

            auto movement = arma::normalise(xb) * dx;

            auto hLeft = visualHorizon.begin();
            auto hRight = visualHorizon.end() - 1;


            // Do our inital calculation to get our first Y
            arma::vec4 worldPosition = arma::ones(4);
            worldPosition.rows(0, 2) = xStart * direction;
            auto camPoint = projectWorldPointToScreen(worldPosition, sensors.orientationCamToGround, FOCAL_LENGTH_PIXELS);
            int y = screenToImage(camPoint, classifiedImage.dimensions)[1];

            for(double x = xStart; x < xEnd && y >= 0; x += std::max(dx, (dx * x) / (cameraHeight - dx))) {

                // Calculate our next Y
                worldPosition.rows(0, 2) = (x + std::max(dx, (dx * x) / (cameraHeight - dx))) * direction;
                camPoint = projectWorldPointToScreen(worldPosition, sensors.orientationCamToGround, FOCAL_LENGTH_PIXELS);
                int nextY = screenToImage(camPoint, classifiedImage.dimensions)[1];

                // Work out our details
                arma::ivec2 start = { 0, y };
                arma::ivec2 end = { int(image.width - 1), y };
                int subsample = std::max(1, int(lround((y - nextY) * BALL_HORIZONTAL_SUBSAMPLE_FACTOR)));

                // If our left hand side is in range, or we are over the top
                if(hLeft->at(1) >= y) {

                    while(hLeft < minHorizon) {

                        auto p1 = hLeft;
                        auto p2 = hLeft + 1;

                        if(y <= p1->at(1) && y >= p2->at(1)) {

                            // Make a line from the two points and find our x
                            Line l({ double(p1->at(0)), double(p1->at(1))}, {double(p2->at(0)), double(p2->at(1))});

                            if(l.isHorizontal()) {
                                start[0] = p2->at(0);
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
                if(hRight->at(1) >= y) {

                    while(hRight > minHorizon) {

                        auto p1 = hRight - 1;
                        auto p2 = hRight;

                        if(y >= p1->at(1) && y <= p2->at(1)) {

                            // Make a line from the two points and find our x
                            Line l({ double(p1->at(0)), double(p1->at(1))}, {double(p2->at(0)), double(p2->at(1))});

                            if(l.isHorizontal()) {
                                end[0] = p1->at(0);
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

                auto segments = quex->classify(image, lut, start, end, subsample);
                insertSegments(classifiedImage, segments, false);
            }

        }

    }  // vision
}  // modules