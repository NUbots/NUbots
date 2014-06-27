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

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::ServoID;
        using messages::input::Sensors;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        using utility::math::geometry::Line;

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
            auto& horizon = classifiedImage.horizon;

            double height = sensors.forwardKinematics.find(ServoID::HEAD_PITCH)->second(3,2) - sensors.forwardKinematics.find(ServoID::L_ANKLE_PITCH)->second(3,2);
            height = 0.4;

            height -= BALL_RADIUS;

            auto hLeft = classifiedImage.maxVisualHorizon;
            auto hRight = classifiedImage.maxVisualHorizon + 1;

            uint kinematicsHorizonPoint = horizon[1];

            for(int p = classifiedImage.minVisualHorizon->at(1) - kinematicsHorizonPoint;
                p + kinematicsHorizonPoint < image.height();
                p = std::max(p + MIN_BALL_SEARCH_JUMP, int(round(1.0 / ((1.0 / double(p)) - ((ALPHA * 2 * BALL_RADIUS) / (MIN_BALL_INTERSECTIONS * height))))))) {

                int y = p + kinematicsHorizonPoint;

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
                    else {
                        --hLeft;
                    }
                }

                while (hRight < visualHorizon.end()) {

                    auto& eq = classifiedImage.visualHorizon[std::distance(visualHorizon.begin(), hRight) - 1];

                    auto p1 = hRight - 1;
                    auto p2 = hRight;

                    if(y >= p1->at(1) && y <= p2->at(1)) {

                        // Make a line from the two points and find our x
                        Line l({ double(p1->at(0)), double(p1->at(1))}, {double(p2->at(0)), double(p2->at(1))});

                        if(l.isHorizontal()) {
                            end[0] = l.getC();
                        }
                        else {
                            end[0] = round(l.findXFromY(y));
                        }

                        break;
                    }
                    // Try our previous point
                    else {
                        ++hRight;
                    }
                }

                auto segments = quex->classify(image, lut, start, end);
                insertSegments(classifiedImage, segments, false);
            }

        }

    }  // vision
}  // modules