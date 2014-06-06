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

        LUTClassifier::LUTClassifier(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            //on<Trigger<Every<10, Per<std::chrono::second>>>([this] (const time_t& t) {});
                        //Load LUTs
            on<Trigger<Configuration<LUTLocations>>>([this](const Configuration<LUTLocations>& config) {
                std::string LUTLocation = config["LUT_LOCATION"];
                emit(std::make_unique<LookUpTable>(LUTLocation));
            });

            on<Trigger<SaveLookUpTable>, With<LookUpTable, Configuration<LUTLocations>>>([this](const SaveLookUpTable&, const LookUpTable& lut, const Configuration<LUTLocations>& config) {
                std::string LUTLocation = config["LUT_LOCATION"];
                lut.save(LUTLocation);
            });

            on<Trigger<Image>, With<LookUpTable, Sensors>, Options<Single>>("Classify Image", [this](const Image& image, const LookUpTable& lut, const Sensors& sensors) {

                auto classifiedImage = std::make_unique<ClassifiedImage<ObjectClass>>();

                // TODO temp, do the horizontal classifications
                for(uint i = 0; i < image.height(); i += 4) {

                    auto segments = m->quex.classify(image, lut, { 0, i }, { image.width() - 1, i });

                    ClassifiedImage<ObjectClass>::Segment* previous = nullptr;
                    ClassifiedImage<ObjectClass>::Segment* current = nullptr;

                    for (auto& s : segments) {

                        // Move in the data
                        current = &(classifiedImage->horizontalSegments.insert(std::make_pair(s.colour, std::move(s)))->second);

                        // Link up the results
                        current->previous = previous;
                        if(previous) {
                            previous->next = current;
                        }

                        // Get ready for our next one
                        previous = current;
                    }
                }

                // TODO temp, do the vertical classifications
                for(uint i = 0; i < image.width(); i += 4) {

                    auto segments = m->quex.classify(image, lut, { i, 0 }, { i, image.height() - 1});

                    ClassifiedImage<ObjectClass>::Segment* previous = nullptr;
                    ClassifiedImage<ObjectClass>::Segment* current = nullptr;

                    for (auto& s : segments) {

                        // Move in the data
                        current = &(classifiedImage->verticalSegments.insert(std::make_pair(s.colour, std::move(s)))->second);

                        // Link up the results
                        current->previous = previous;
                        if(previous) {
                            previous->next = current;
                        }

                        // Get ready for our next one
                        previous = current;
                    }
                }

                emit(std::move(classifiedImage));

                // classify every 4th line vert/horizontal

                // We need a quex classifier object
                // We need a way of passing in the LUTified data to the classifer object

                // Make a classified image
                //auto classifiedimage out = std::make_unique<ClassifiedImage>();

                // Cast lines to find the green horizon
                //      Get the kinematics horizon
                //      Cast lines from slightly above it to the bottom of the image
                //      Cast them at a width that will find a ball at the bottom of the image
                //      Convex hull the green horizon

                // Cast lines down from the green horizon to find the ball at varying depths
                //      Calculate the line distances given kinematics etc...
                //      Cast the lines and classify

                // Cast horizontal lines to find the goals
                //      Cast horizontal lines above the green horizon and slightly below

                // Cross hatch the ball locations
                //      Get the midpoints for orange
                //      do a p-norm the midpoints to find a "centre"
                //      Using the expected ball size for that distance cross hatch around to cover

                // Cross hatch the goal locations
                //      Get the transition points for yellow
                //      using the segments, ensure that there are horizontal hatches along the goal height
                //      find the lowest and highest pairs and hatch around it to find the bottoms and tops
                //      cast several close horizontal lines at the tops to find the crossbar
                //      Cast vertical lines to finish the crossbar

                /*for(strategy : strategies) {
                    std::vector<segment> a = strategy1.request(*classifiedimage);
                    segments = quex.classify(image, lut, a);
                    strategy1.resolve(segments, classifiedImage);
                }*/



                // Stage 1 - Find green horizon

                // Stage 2 - Do the logrithmic ball finder

                // Stage 2/3 - Do the horizontal goal finder

                // Stage 4 - Crosshatch
            });

        }

    }  // vision
}  // modules