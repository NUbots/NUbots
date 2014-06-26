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

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::ServoID;
        using messages::input::Sensors;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

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

            auto& horizonPoints = classifiedImage.visualHorizonPoints;
            auto& horizon = classifiedImage.horizon;

            double height = sensors.forwardKinematics.find(ServoID::HEAD_PITCH)->second(3,2) - sensors.forwardKinematics.find(ServoID::L_ANKLE_PITCH)->second(3,2);
            height = 0.4;

            height -= BALL_RADIUS;

            // Get the visual horizon intercepts for this point (either side)
            auto maxPoint = std::min_element(horizonPoints.begin(), horizonPoints.end(), [] (const arma::uvec2& a, const arma::uvec2& b) {
                return a[1] < b[1];
            });
            auto hLeft = maxPoint;
            auto hRight = maxPoint + 1;

            uint kinematicsHorizonPoint = horizon[1];

            for(int p = classifiedImage.minVisualHorizonPoint->at(1) - kinematicsHorizonPoint;
                p + kinematicsHorizonPoint < image.height();
                p = std::max(p + MIN_BALL_SEARCH_JUMP, int(round(1.0 / ((1.0 / double(p)) - ((ALPHA * 2 * BALL_RADIUS) / (MIN_BALL_INTERSECTIONS * height))))))) {

                uint y = p + kinematicsHorizonPoint;

                arma::uvec2 start = { 0, y };
                arma::uvec2 end = { image.width() - 1, y };

                while (hLeft > horizonPoints.begin()) {

                    auto& eq = classifiedImage.visualHorizon[std::distance(horizonPoints.begin(), hLeft) - 1];

                    double y1 = (hLeft - 1)->at(1);
                    double y2 = hLeft->at(1);

                    if(y <= y1 && y >= y2 && eq[1] != 0) {

                        // Solve the equation for x
                        start[0] = std::round((y - eq[2]) / eq[1]);
                        break;
                    }
                    // Try our previous point
                    else {
                        --hLeft;
                    }
                }

                while (hRight < horizonPoints.end()) {

                    auto& eq = classifiedImage.visualHorizon[std::distance(horizonPoints.begin(), hRight) - 1];

                    double y1 = (hRight - 1)->at(1);
                    double y2 = hRight->at(1);

                    if(y >= y1 && y <= y2 && eq[1] != 0) {

                        // Solve the equation for x
                        end[0] = std::round((y - eq[2]) / eq[1]);
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