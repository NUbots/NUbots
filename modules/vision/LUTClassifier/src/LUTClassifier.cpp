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
#include "utility/idiom/pimpl_impl.h"

#include "messages/input/Image.h"
#include "messages/input/CameraParameters.h"
#include "messages/input/Sensors.h"
#include "messages/vision/LookUpTable.h"
#include "messages/vision/SaveLookUpTable.h"
#include "messages/support/Configuration.h"

#include "QuexClassifier.h"

#include "Lexer.hpp"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::ServoID;
        using messages::input::Sensors;
        using messages::input::CameraParameters;
        using messages::vision::LookUpTable;
        using messages::vision::SaveLookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;
        using messages::support::Configuration;
        using messages::support::SaveConfiguration;

        // Implement our impl class.
        class LUTClassifier::impl {
        public:
            QuexClassifier quex;

            uint VISUAL_HORIZON_SPACING = 100;
            uint VISUAL_HORIZON_BUFFER = 0;
            uint MINIMUM_VISUAL_HORIZON_SEGMENT_SIZE = 0;
            uint VISUAL_HORIZON_SUBSAMPLING = 1;
            uint GOAL_FINDER_LINE_SPACING = 100;
            uint GOAL_FINDER_SUBSAMPLING = 1;
            std::vector<double> GOAL_FINDER_DETECTOR_LEVELS = { 2.0 };
            double BALL_SEARCH_FACTOR = 2.0;
            int MIN_BALL_SEARCH_JUMP = 1;

            void setParameters(const CameraParameters& cam, const Configuration<LUTClassifier>& config) {
                // Visual horizon detector
                VISUAL_HORIZON_SPACING = cam.effectiveScreenDistancePixels * tan(config["visual_horizon"]["spacing"].as<double>());
                VISUAL_HORIZON_BUFFER = cam.effectiveScreenDistancePixels * tan(config["visual_horizon"]["horizon_buffer"].as<double>());
                VISUAL_HORIZON_SUBSAMPLING = std::max(1, int(cam.effectiveScreenDistancePixels * tan(config["visual_horizon"]["subsampling"].as<double>())));
                MINIMUM_VISUAL_HORIZON_SEGMENT_SIZE = cam.effectiveScreenDistancePixels * tan(config["visual_horizon"]["minimum_segment_size"].as<double>());

                // // Goal detector
                GOAL_FINDER_LINE_SPACING = cam.effectiveScreenDistancePixels * tan(config["goals"]["spacing"].as<double>());
                GOAL_FINDER_SUBSAMPLING = std::max(1, int(cam.effectiveScreenDistancePixels * tan(config["goals"]["subsampling"].as<double>())));
                GOAL_FINDER_DETECTOR_LEVELS = config["goals"]["detector_levels"].as<std::vector<double>>();

                // Halve our levels
                for(auto& d : GOAL_FINDER_DETECTOR_LEVELS) {
                    d /= 2;
                }

                // // Ball Detector
                double minIntersections = config["ball"]["intersections"].as<double>();
                BALL_SEARCH_FACTOR = 2 * 0.1 * cam.pixelsToTanThetaFactor[1] / minIntersections;
                MIN_BALL_SEARCH_JUMP = std::max(1, int(cam.effectiveScreenDistancePixels * tan(config["ball"]["min_jump"].as<double>())));
            }
        };

        void insertSegments(ClassifiedImage<ObjectClass>& image, std::vector<ClassifiedImage<ObjectClass>::Segment>& segments, bool vertical) {
            ClassifiedImage<ObjectClass>::Segment* previous = nullptr;
            ClassifiedImage<ObjectClass>::Segment* current = nullptr;

            auto& target = vertical ? image.verticalSegments : image.horizontalSegments;

            for (auto& s : segments) {

                // Move in the data
                current = &(target.insert(std::make_pair(s.colour, std::move(s)))->second);

                // Link up the results
                current->previous = previous;
                if(previous) {
                    previous->next = current;
                }

                // Get ready for our next one
                previous = current;
            }
        }

        LUTClassifier::LUTClassifier(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<Configuration<LUTLocation>>>([this](const Configuration<LUTLocation>& config) {
                emit(std::make_unique<LookUpTable>(config.config.as<LookUpTable>()));
            });

            on<Trigger<SaveLookUpTable>, With<LookUpTable>>([this](const SaveLookUpTable&, const LookUpTable& lut) {
                emit(std::make_unique<SaveConfiguration>(SaveConfiguration{ LUTLocation::CONFIGURATION_PATH, YAML::Node(lut) }));
            });

            auto setParams = [this] (const CameraParameters& cam, const Configuration<LUTClassifier>& config) {
                m->setParameters(std::forward<const CameraParameters&>(cam), std::forward<const Configuration<LUTClassifier>&>(config));
            };

            on<Trigger<CameraParameters>, With<Configuration<LUTClassifier>>>(setParams);
            on<With<CameraParameters>, Trigger<Configuration<LUTClassifier>>>(setParams);

            on<Trigger<Image>, With<LookUpTable, Sensors>, Options<Single>>("Classify Image", [this](const Image& image, const LookUpTable& lut, const Sensors& sensors) {

                auto classifiedImage = std::make_unique<ClassifiedImage<ObjectClass>>();

                /**********************************************
                 *                FIND HORIZON                *
                 **********************************************/

                // Element 0 is gradient, element 1 is intercept (confirmed by Jake's Implementation)
                // Coordinate system: 0,0 is the centre of the screen. pos[0] is along the y axis of the
                // camera transform, pos[1] is along the z axis (x points out of the camera)
                arma::vec2 horizon = sensors.kinematicsHorizon;

                // Move the intercept to be at 0,0
                horizon[1] += (image.height() * 0.5) + horizon[0] * -(image.width() * 0.5);

                // This is our new images horizon
                classifiedImage->horizon = horizon;

                /**********************************************
                 *             FIND VISUAL HORIZON            *
                 **********************************************/

                std::vector<arma::uvec2> horizonPoints;

                // Cast lines to find our visual horizon

                // TODO enforce that a line is cast on the right hand side of the image
                for(uint i = 0; i < image.width(); i += m->VISUAL_HORIZON_SPACING) {

                    // Find our point to classify from (slightly above the horizon)
                    uint top = std::max(int(i * horizon[0] + horizon[1] - m->VISUAL_HORIZON_BUFFER), int(0));
                    top = std::min(top, image.height() - 1);

                    // Classify our segments
                    auto segments = m->quex.classify(image, lut, { i, top }, { i, image.height() - 1 });

                    // Our default green point is the bottom of the screen
                    arma::uvec2 greenPoint = { i, image.height() - 1 };

                    // Loop through our segments backward (top to bottom) to find our first green segment
                    for (auto it = segments.begin(); it != segments.end(); ++it) {

                        // If this a valid green point update our information
                        if(it->colour == ObjectClass::FIELD && it->length >= m->MINIMUM_VISUAL_HORIZON_SEGMENT_SIZE) {
                            greenPoint = it->start;
                            // We found our green
                            break;
                        }
                    }

                    horizonPoints.push_back(std::move(greenPoint));

                    insertSegments(*classifiedImage, segments, true);
                }

                // Do a convex hull on the map points to build the horizon
                for(auto a = horizonPoints.begin(); a < horizonPoints.end() - 2;) {

                    auto b = a + 1;
                    auto c = a + 2;

                    // Get the Z component of a cross product to check if it is concave
                    bool concave = 0 <   (double(a->at(0)) - double(b->at(0))) * (double(c->at(1)) - double(b->at(1)))
                                       - (double(a->at(1)) - double(b->at(1))) * (double(c->at(0)) - double(b->at(0)));

                    if(concave) {
                        horizonPoints.erase(b);
                        a = a == horizonPoints.begin() ? a : --a;
                    }
                    else {
                        ++a;
                    }
                }

                uint maxVisualHorizon = 0;
                uint minVisualHorizon = image.height();

                for(uint i = 0; i < horizonPoints.size() - 1; ++i) {
                    const auto& p1 = horizonPoints[i];
                    const auto& p2 = horizonPoints[i + 1];

                    maxVisualHorizon = std::max({ maxVisualHorizon, uint(p1[1]), uint(p2[1]) });
                    minVisualHorizon = std::min({ minVisualHorizon, uint(p1[1]), uint(p2[1]) });

                    double m = (double(p2[1]) - double(p1[1])) / (double(p2[0]) - double(p1[0]));
                    double b = - m * double(p1[0]) + double(p1[1]);

                    classifiedImage->visualHorizon.push_back({ double(p1[0]), m, b });
                }

                /**********************************************
                 *           CAST BALL FINDER LINES           *
                 **********************************************/

                /*
                    Here we cast lines to find balls.
                    To do this, we cast lines seperated so that any ball will have at least 2 lines
                    passing though it (possibly 3).
                    This means that lines get logrithmically less dense as we decend the image as a balls
                    apparent size will be larger.
                    These lines are cast from slightly above the visual horizon to a point where it is needed
                    (for the logrithmic grid)
                 */

                // Update equation
                /// gives between l and l+1 lines through ball
                ///
                /// l        = min lines through ball
                /// r        = radius of ball
                /// h        = robot's camera height
                /// p        = number of pixels below kinematics horizion
                /// $\alpha$ = pixels to tan(\theta) ratio
                ///
                /// $\Delta p=p^{2}\frac{2r\alpha}{lh}$

                double height = sensors.forwardKinematics.find(ServoID::HEAD_PITCH)->second(3,2) - sensors.forwardKinematics.find(ServoID::L_ANKLE_PITCH)->second(3,2);
                double eqConst = m->BALL_SEARCH_FACTOR / 0.4;

                // Get the visual horizon intercepts for this point (either side)
                auto maxPoint = std::min_element(horizonPoints.begin(), horizonPoints.end(), [] (const arma::uvec2& a, const arma::uvec2& b) {
                    return a[1] < b[1];
                });
                auto hLeft = maxPoint;
                auto hRight = maxPoint + 1;

                uint kinematicsHorizonPoint = horizon[1];

                for(int p = minVisualHorizon - kinematicsHorizonPoint; p + kinematicsHorizonPoint < image.height(); p += std::max(int(p * p * eqConst), m->MIN_BALL_SEARCH_JUMP)) {

                    uint y = p + kinematicsHorizonPoint;

                    arma::uvec2 start = { 0, y };
                    arma::uvec2 end = { image.width() - 1, y };

                    while (hLeft > horizonPoints.begin()) {

                        auto& eq = classifiedImage->visualHorizon[std::distance(horizonPoints.begin(), hLeft) - 1];

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

                        auto& eq = classifiedImage->visualHorizon[std::distance(horizonPoints.begin(), hRight) - 1];

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

                    auto segments = m->quex.classify(image, lut, start, end);
                    insertSegments(*classifiedImage, segments, false);
                }

                /**********************************************
                 *           CAST GOAL FINDER LINES           *
                 **********************************************/

                /*
                   Here we cast classification lines to attempt to locate the general area of the goals.
                   We cast lines only above the visual horizon (with some buffer) so that we do not over.
                   classify the mostly empty green below.
                 */

                // Using yellow segments found by the ball finder, draw some vertical lines to find the bottom of the goals
                // TODO implement

                // Reset our hMax and hMin so we can do the oppisite check for the ball (to search outside the horizon)
                // The variables should already be set to this point... but just to be sure
                hLeft = horizonPoints.begin();
                hRight = horizonPoints.end() - 1;

                // Cast lines upward to find the goals
                for(int y = maxVisualHorizon; y >= 0; y -= m->GOAL_FINDER_LINE_SPACING) {

                    // If our left hand side is in range, or we are over the top
                    if(hLeft->at(1) >= uint(y)) {

                        arma::uvec2 start = { uint(0), uint(y) };
                        arma::uvec2 end = { image.width() - 1, uint(y) };

                        while(hLeft < maxPoint) {

                            auto& eq = classifiedImage->visualHorizon[std::distance(horizonPoints.begin(), hLeft)];

                            int y1 = hLeft->at(1);
                            int y2 = (hLeft + 1)->at(1);

                            if(y <= y1 && y >= y2 && eq[1] != 0) {

                                // Solve the equation for x
                                end[0] = std::round((double(y) - eq[2]) / eq[1]);
                                break;
                            }
                            // Try our previous point
                            else {
                                ++hLeft;
                            }
                        }

                        // Insert our segments
                        auto segments = m->quex.classify(image, lut, start, end);
                        insertSegments(*classifiedImage, segments, false);
                    }

                    // If our right hand side is in range and has not gone out of scope
                    if(hRight->at(1) >= y && hRight > maxPoint) {

                        arma::uvec2 start = { uint(0), uint(y) };
                        arma::uvec2 end = { image.width() - 1, uint(y) };

                        while(hRight > maxPoint) {

                            auto& eq = classifiedImage->visualHorizon[std::distance(horizonPoints.begin(), hRight) - 1];

                            int y1 = (hRight - 1)->at(1);
                            int y2 = hRight->at(1);

                            if(y >= y1 && y <= y2 && eq[1] != 0) {

                                // Solve the equation for x
                                start[0] = std::round((double(y) - eq[2]) / eq[1]);
                                break;
                            }
                            // Try our previous point
                            else {
                                --hRight;
                            }
                        }

                        // Insert our segments
                        auto segments = m->quex.classify(image, lut, start, end);
                        insertSegments(*classifiedImage, segments, false);

                    }
                }

                /**********************************************
                 *              CROSSHATCH BALLS              *
                 **********************************************/

                /*
                    This section improves the classification of the ball.
                    We first find all of the orange transitions that are below the visual horizon.
                    We then take the norm of these points to attempt to find a very rough "centre" for the ball.
                    Using the expected size of the ball at this position on the screen, we then crosshatch 2x the
                    size needed to ensure that the ball is totally covered.
                 */
                arma::running_stat_vec<arma::uvec> centre;
                for(auto it = classifiedImage->horizontalSegments.lower_bound(ObjectClass::BALL);
                    it != classifiedImage->horizontalSegments.upper_bound(ObjectClass::BALL);
                    ++it) {

                    auto& elem = it->second;

                    centre(elem.midpoint);

                    // Get the expected size of the ball at the
                }

                // Find the size of a ball at the position
                auto ballSize = centre.mean();

                // Distance to point to centre ( n below horizon = h/alphax )

                // Get the width of the imaginary ball

                // Find the angular width of the ball at this distance

                // Multiply tan of that angle by the angle->pixels constant (alpha?)

                /**********************************************
                 *              CROSSHATCH GOALS              *
                 **********************************************/

                /*
                    Here we improve the classification of goals.
                    We do this by taking our course classification of the whole image
                    and generating new segments where yellow was detected.
                    We first generate segments above and below that are 2x the width of the segment
                    We then take these segments and generate segments that are 1.2x the width
                    This should allow a high level of detail without overclassifying the image
                 */

                for (uint i = 0; i < m->GOAL_FINDER_DETECTOR_LEVELS.size(); ++i) {

                    std::vector<ClassifiedImage<ObjectClass>::Segment> newSegments;

                    for(auto it = classifiedImage->horizontalSegments.lower_bound(ObjectClass::GOAL);
                        it != classifiedImage->horizontalSegments.upper_bound(ObjectClass::GOAL);
                        ++it) {

                        auto& elem = it->second;
                        arma::vec2 midpoint = arma::conv_to<arma::vec>::from(elem.midpoint);

                        arma::vec upperBegin = midpoint + arma::vec({ -double(elem.length) * m->GOAL_FINDER_DETECTOR_LEVELS[i],  double(m->GOAL_FINDER_LINE_SPACING) / std::pow(3, i + 1) });
                        arma::vec upperEnd   = midpoint + arma::vec({  double(elem.length) * m->GOAL_FINDER_DETECTOR_LEVELS[i],  double(m->GOAL_FINDER_LINE_SPACING) / std::pow(3, i + 1) });
                        arma::vec lowerBegin = midpoint + arma::vec({ -double(elem.length) * m->GOAL_FINDER_DETECTOR_LEVELS[i], -double(m->GOAL_FINDER_LINE_SPACING) / std::pow(3, i + 1) });
                        arma::vec lowerEnd   = midpoint + arma::vec({  double(elem.length) * m->GOAL_FINDER_DETECTOR_LEVELS[i], -double(m->GOAL_FINDER_LINE_SPACING) / std::pow(3, i + 1) });

                        upperBegin[0] = std::max(upperBegin[0], double(0));
                        upperBegin[0] = std::min(upperBegin[0], double(image.width() - 1));

                        upperEnd[0] = std::max(upperEnd[0], double(0));
                        upperEnd[0] = std::min(upperEnd[0], double(image.width() - 1));

                        lowerBegin[0] = std::max(lowerBegin[0], double(0));
                        lowerBegin[0] = std::min(lowerBegin[0], double(image.width() - 1));

                        lowerEnd[0] = std::max(lowerEnd[0], double(0));
                        lowerEnd[0] = std::min(lowerEnd[0], double(image.width() - 1));

                        // If the upper segment is valid
                        if(upperBegin[0] != upperEnd[0]
                          && (upperBegin[1] < image.height() && upperBegin[1] >= 0)
                          && (upperEnd[1] < image.height() && upperEnd[1] >= 0)) {

                            auto segments = m->quex.classify(image, lut, arma::conv_to<arma::uvec>::from(upperBegin), arma::conv_to<arma::uvec>::from(upperEnd));

                            newSegments.insert(newSegments.begin(), segments.begin(), segments.end());
                        }

                        // If the lower segment is valid and not the same as the upper segment
                        if(lowerBegin[0] != lowerEnd[0]
                          && (lowerBegin[1] < image.height() && lowerBegin[1] >= 0)
                          && (lowerEnd[1] < image.height() && lowerEnd[1] >= 0)) {

                            auto segments = m->quex.classify(image, lut, arma::conv_to<arma::uvec>::from(lowerBegin), arma::conv_to<arma::uvec>::from(lowerEnd));

                            newSegments.insert(newSegments.begin(), segments.begin(), segments.end());
                        }
                    }

                    insertSegments(*classifiedImage, newSegments, false);
                }

                emit(std::move(classifiedImage));
            });

        }

    }  // vision
}  // modules