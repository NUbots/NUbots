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
#include "messages/input/Sensors.h"
#include "messages/vision/LookUpTable.h"
#include "messages/vision/SaveLookUpTable.h"
#include "messages/support/Configuration.h"

#include "QuexClassifier.h"

#include "Lexer.hpp"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::Sensors;
        using messages::vision::LookUpTable;
        using messages::vision::SaveLookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;
        using messages::support::Configuration;

        // Implement our impl class.
        class LUTClassifier::impl {
            public:
            QuexClassifier quex;
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

            //on<Trigger<Every<10, Per<std::chrono::second>>>([this] (const time_t& t) {});
                        //Load LUTs
            on<Trigger<Configuration<LUTLocation>>>([this](const Configuration<LUTLocation>& config) {
                emit(std::make_unique<LookUpTable>(config.config.as<LookUpTable>()));
            });

            on<Trigger<Image>, With<LookUpTable, Sensors>, Options<Single>>("Classify Image", [this](const Image& image, const LookUpTable& lut, const Sensors& sensors) {

                // TODO this should be wide enough to ensure a really close ball has at least 2 lines passing thorugh it
                // It should also ensure that the 0th column and last column is done
                constexpr uint VISUAL_HORIZON_SPACING = 10;
                constexpr uint HORIZON_BUFFER = 10;
                constexpr uint MINIMUM_VISUAL_HORIZON_SEGMENT_SIZE = 5;
                constexpr uint GOAL_LINE_SPACING = 5;

                auto classifiedImage = std::make_unique<ClassifiedImage<ObjectClass>>();

                /**********************************************
                 *                FIND HORIZON                *
                 **********************************************/

                // Get our actual horizon
                arma::vec2 horizon = {10, 10}; // Element 0 is gradient, element 1 is intercept
                classifiedImage->horizon = horizon;


                /**********************************************
                 *             FIND VISUAL HORIZON            *
                 **********************************************/

                // Cast lines to find our visual horizon
                for(uint i = 0; i < image.width(); i += VISUAL_HORIZON_SPACING) {

                    // Find our point to classify from (slightly above the horizon)
                    uint top = std::min(uint(i * horizon[0] + horizon[1] + HORIZON_BUFFER), uint(image.height()));

                    // Classify our segments
                    auto segments = m->quex.classify(image, lut, { i, 0 }, { i, top });
                    insertSegments(*classifiedImage, segments, true);
                }

                // Find our candidate points for the horizon
                std::map<uint, uint> points;
                for(auto it = classifiedImage->verticalSegments.lower_bound(ObjectClass::FIELD);
                    it != classifiedImage->verticalSegments.upper_bound(ObjectClass::FIELD);
                    ++it)

                    // If this segment is large enough
                    if(it->second.length > MINIMUM_VISUAL_HORIZON_SEGMENT_SIZE) {

                        // Add the point if it's larger then the current one
                    }
                }

                // Do a convex hull on the map points to build the horizon



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

                // Based on the horizon and level get an end point

                // line starts at visual horizon + buffer down to end point


                /**********************************************
                 *           CAST GOAL FINDER LINES           *
                 **********************************************/

                /*
                   Here we cast classification lines to attempt to locate the general area of the goals.
                   We cast lines only above the visual horizon (with some buffer) so that we do not over.
                   classify the mostly empty green below.
                 */
                uint lowerBound = std::min(horizon[1], horizon[0] * image.width() + horizon[1]);
                for(uint i = lowerBound; i > image.height(); i += GOAL_LINE_SPACING) {

                    // Cast a full horizontal line here
                    auto segments = m->quex.classify(image, lut, { 0, i }, { image.width(), i });
                    insertSegments(*classifiedImage, segments, true);
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
                for(auto it = classifiedImage.horizontalSegments.lower_bound(ObjectClass::GOAL);
                    it != classifiedImage.horizontalSegments.upper_bound(ObjectClass::GOAL);
                    ++it) {

                    auto& elem = it->second;

                    // Get the new points to classify above
                    arma::vec2 begin = elem.midpoint + arma::vec2({ GOAL_LINE_SPACING / 3,  elem.length });
                    arma::vec2 end = elem.midpoint + arma::vec2({ GOAL_LINE_SPACING / 3, -elem.length });;

                    // Classify the new segments above
                    auto segments = m->quex.classify(image, lut, begin, end);

                    // Get the new points to classify below
                    begin = elem.midpoint + arma::vec2({ -GOAL_LINE_SPACING / 3,  elem.length });
                    end = elem.midpoint + arma::vec2({ -GOAL_LINE_SPACING / 3, -elem.length });;

                    // Classify the new segments below
                    auto segments = m->quex.classify(image, lut, begin, end);
                }

                // Do the same thing again, with a finer grain and only 1.2x the size
                for(auto it = classifiedImage.horizontalSegments.lower_bound(ObjectClass::GOAL);
                    it != classifiedImage.horizontalSegments.upper_bound(ObjectClass::GOAL);
                    ++it) {

                    auto& elem = it->second;

                    // Get the new points to classify above
                    arma::vec2 begin = elem.midpoint + arma::vec2({ GOAL_LINE_SPACING / 9,  elem.length * 0.6 });
                    arma::vec2 end = elem.midpoint + arma::vec2({ GOAL_LINE_SPACING / 9, -elem.length * 0.6 });;

                    // Classify the new segments above
                    auto segments = m->quex.classify(image, lut, begin, end);

                    // Get the new points to classify below
                    begin = elem.midpoint + arma::vec2({ -GOAL_LINE_SPACING / 9,  elem.length * 0.6 });
                    end = elem.midpoint + arma::vec2({ -GOAL_LINE_SPACING / 9, -elem.length * 0.6 });;

                    // Classify the new segments below
                    auto segments = m->quex.classify(image, lut, begin, end);
                }


                // For each yellow segment of large enough size
                // cast a line N above and N below that is 2x the width of this segment

                // Iterate through this n times

                emit(std::move(classifiedImage));
            });

        }

    }  // vision
}  // modules