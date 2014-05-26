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
#include "messages/input/Image.h"
#include "messages/input/Sensors.h"
#include "messages/vision/LookUpTable.h"
#include "Lexer.hpp"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::Sensors;
        using messages::vision::LookUpTable;

        LUTClassifier::LUTClassifier(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            //on<Trigger<Every<10, Per<std::chrono::second>>>([this] (const time_t& t) {});

            on<Trigger<Image>, With<LookUpTable, Sensors>, Options<Single>>([this](const Image& image, const LookUpTable& lut, const Sensors& sensors) {


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