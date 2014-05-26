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

#include "QuexClassifier.h"

namespace modules {
    namespace vision {
        using messages::input::Image;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        QuexClassifier() {
        };

        std::multimap<ObjectClass, ClassifiedImage::Segment> QuexClassifier::classify(const Image& image, const LookUpTable& lut, const arma::vec2& start, const arma::vec2& end) {

            // Reset our buffer

            // Reset the quex lexer


            // Start reading data
            classifier.buffer_fill_region_prepare();

            // For horizontal runs
            if(start[0] == end[0]) {

                size_t length = end[1] - start[1];
                ensureCapacity(length);

                for(uint i = start[1]; i < end[1]; ++i) {

                    char v = lut.classify(image(start[0], i));
                }

                classifer.buffer_fill_region_finish(length);
            }

            // For vertical runs
            else if(start[1] == end[1]) {

                size_t length = end[0] - start[0];
                ensureCapacity(length);

                for(uint i = start[0]; i < end[0]; ++i) {

                    char v = lut.classify(image(i, start[1]));
                }

                classifer.buffer_fill_region_finish(length);
            }

            // Diagonal run
            else {
                // TODO not implemented
            }

            // Read our lexing tokens
            do {
                Token token;
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

        }
    }
}