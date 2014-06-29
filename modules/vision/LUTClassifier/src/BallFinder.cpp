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

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::ServoID;
        using messages::input::Sensors;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        using utility::math::geometry::Line;
        using utility::math::vision::getGroundPointFromScreen;
        using utility::math::vision::projectWorldPointToCamera;

        void LUTClassifier::findBall(const Image& image, const LookUpTable& lut, const Sensors& sensors, ClassifiedImage<ObjectClass>& classifiedImage) {

            /*
                Here we cast lines to find balls.
                To do this, we cast lines seperated so that any ball will have at least 2 lines
                passing though it (possibly 3).
                This means that lines get logrithmically less dense as we decend the image as a balls
                apparent size will be larger.
                These lines are cast from slightly above the visual horizon to a point where it is needed
                (for the logrithmic grid)
             */

            // Update equation: p_{n+1}=\frac{h}{\frac{h}{p_{n}}-\frac{\alpha r}{\sin \left( \mbox{atan}\left( \alpha p_{n} \right) \right)}}
            /// gives between l and l+1 lines through ball
            ///
            /// l        = min lines through ball
            /// r        = radius of ball
            /// h        = robot's camera height
            /// p        = number of pixels below kinematics horizion
            /// $\alpha$ = pixels to tan(\theta) ratio
            ///
            /// $\Delta p=p^{2}\frac{2r\alpha}{lh}$

            auto& visualHorizon = classifiedImage.visualHorizon;

            double topY = -(classifiedImage.minVisualHorizon->at(1) - double(image.height() / 2));
          //std::cout  << "topY" << topY << std::endl;

            // Get the positions of the top of our green horizion, and the bottom of the screen
            auto xb = getGroundPointFromScreen({ 0, -double(image.height() / 2)}, sensors.kinematicsCamToGround, FOCAL_LENGTH_PIXELS);
            auto xt = getGroundPointFromScreen({ 0, topY}, sensors.kinematicsCamToGround, FOCAL_LENGTH_PIXELS);
            double dx = 2 * BALL_RADIUS / MIN_BALL_INTERSECTIONS;
            double cameraHeight = sensors.kinematicsCamToGround(2,3);
          //std::cout  << "xb" << xb.t() << std::endl;
          //std::cout  << "xt" << xt.t() << std::endl;
            

            // This describes the direction of travel
            auto direction = arma::normalise(xb);

            // Our
            double xStart = arma::norm(xb);
            xStart += dx - fmod(xStart, dx);
            double xEnd = arma::norm(xt);

          //std::cout  << "xStart " << xStart << std::endl;
          //std::cout  << "xEnd " << xEnd << std::endl;
          //std::cout  << "dx " << dx << std::endl;

            auto movement = arma::normalise(xb) * dx;

            auto hLeft = classifiedImage.minVisualHorizon;
            auto hRight = classifiedImage.minVisualHorizon + 1;

            for(double x = xStart; x < xEnd; x += std::max(dx, (dx * x) / (cameraHeight - dx)))  {

                arma::vec4 worldPosition = arma::ones(4);

                worldPosition.rows(0, 2) = x * direction;

                // Transform x onto the camera
                auto camPoint = projectWorldPointToCamera(worldPosition, sensors.kinematicsCamToGround, FOCAL_LENGTH_PIXELS);
              //std::cout  << "worldPosition" << worldPosition.t() << std::endl;
              //std::cout  << "camPoint" << camPoint.t() << std::endl;
                // Transform into our coordinates
                int y = lround(-camPoint[1] + (image.height() / 2));
                
              //std::cout  << "y" << y << std::endl;
                arma::ivec2 start = { 0, y };
                arma::ivec2 end = { int(image.width() - 1), y };

                while (hLeft > visualHorizon.begin()) {

                    auto p1 = hLeft - 1;
                    auto p2 = hLeft;

                    if(y <= p1->at(1) && y >= p2->at(1)) {

                        // Make a line from the two points and find our x
                        Line l({ double(p1->at(0)), double(p1->at(1))}, {double(p2->at(0)), double(p2->at(1))});

                        if(l.isHorizontal()) {
                            start[0] = l.getC();
                        }
                        else {
                            start[0] = round(l.findXFromY(y));
                        }

                        break;
                    }

                    // Try our previous point
                    --hLeft;
                }

                while (hRight < visualHorizon.end()) {

                    auto p1 = hRight - 1;
                    auto p2 = hRight;

                    if(y >= p1->at(1) && y <= p2->at(1)) {

                        // Make a line from the two points and find our x
                        Line l({ double(p1->at(0)), double(p1->at(1))}, {double(p2->at(0)), double(p2->at(1))});

                        if(!l.isHorizontal()) {
                            end[0] = round(l.findXFromY(y));
                            break;
                        }
                    }

                    // Try our previous point
                    ++hRight;
                }

                auto segments = quex->classify(image, lut, start, end);
                insertSegments(classifiedImage, segments, false);
            }

        }

    }  // vision
}  // modules