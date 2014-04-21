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

#include "QuexLUTClassifier.h"
#include "messages/input/Image.h"
#include "messages/vision/LookUpTable.h"
#include "Replacement.hpp"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::vision::LookUpTable;

        QuexLUTClassifier::QuexLUTClassifier(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            //on<Trigger<Every<10, Per<std::chrono::second>>>([this] (const time_t& t) {});

            on<Trigger<Image>, With<LookUpTable, std::vector<ScanStrategy>>, Options<Single>>([this](const Image& image, const LookUpTable& lut) {



                for(strategy : strategies) {
                    std::vector<segment> a = strategy1.request(classifiedimage);
                    segments = quex.classify(image, lut, a);
                    strategy1.resolve(segments, classifiedImage);
                }

                emit(classifiedimage);


                // Stage 1 - Find green horizon

                // Stage 2 - Do the logrithmic ball finder

                // Stage 2/3 - Do the horizontal goal finder

                // Stage 4 - Crosshatch

                // Heap allocate the most memory we will need
                std::vector<char> in;
                in.reserve(std::max(image.width(), image.height()));

                // Make our classifier object
                quex::Replacement classifier(in.data(), in.size(), in.data() + 1);

                classifier.buffer_fill_region_prepare();

                // Write to our buffer

                classifer.buffer_fill_region_finish(numofchars);

                do {
                    qlex.token_p_switch(&token);
                    qlex.receive();

                    switch(token.type_id) {
                        case QUEX_TKN_BALL:
                            //token.number;
                            break;

                        case QUEX_TKN_CYAN_TEAM:
                            //token.number;
                            break;

                        case QUEX_TKN_FIELD:
                            //token.number;
                            break;

                        case QUEX_TKN_GOAL:
                            //token.number;
                            break;

                        case QUEX_TKN_LINE:
                            //token.number;
                            break;

                        case QUEX_TKN_MAGENTA_TEAM:
                            //token.number;
                            break;

                        case QUEX_TKN_UNCLASSIFIED:
                            //token.number;
                            break;
                }
                while(token.type_id() != QUEX_TKN_TERMINATION);
            });

        }

    }  // vision
}  // modules