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

#include <iostream>

namespace modules {
    namespace vision {
        using messages::input::Image;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;
        using quex::Token;

        QuexClassifier::QuexClassifier() : lexer(buffer, BUFFER_SIZE, buffer + 1) {
        }

        std::multimap<ObjectClass, ClassifiedImage<ObjectClass>::Segment> QuexClassifier::classify(const Image& image, const LookUpTable& lut, const arma::vec2& start, const arma::vec2& end) {

            // Start reading data
            lexer.buffer_fill_region_prepare();

            // For vertical runs
            if(start[0] == end[0]) {

                size_t length = end[1] - start[1] + 1;

                for(uint i = 0; i < length; ++i) {
                    buffer[i + 1] = lut.classify(image(start[0], start[1] + i));
                }

                lexer.buffer_fill_region_finish(length);
            }
            // For horizontal runs
            else if(start[1] == end[1]) {

                size_t length = end[0] - start[0] + 1;

                for(uint i = 0; i < length; ++i) {
                    buffer[i + 1] = lut.classify(image(start[0] + 1, start[1]));
                }

                lexer.buffer_fill_region_finish(length - 1);
                lexer.buffer_input_pointer_set(buffer + 1);
            }

            // Diagonal run
            else {
                // TODO not implemented
            }

            // Read our lexing tokens
            Token token;
            do {
                lexer.token_p_switch(&token);
                lexer.receive();

                switch(token.type_id()) {
                    case QUEX_TKN_BALL:
                        std::cout << "Ball: " << token.number << std::endl;
                        break;

                    case QUEX_TKN_CYAN_TEAM:
                        std::cout << "Cyan: " << token.number << std::endl;
                        break;

                    case QUEX_TKN_FIELD:
                        std::cout << "Field: " << token.number << std::endl;
                        break;

                    case QUEX_TKN_GOAL:
                        std::cout << "Goal: " << token.number << std::endl;
                        break;
                    case QUEX_TKN_LINE:
                        std::cout << "Line: " << token.number << std::endl;
                        break;

                    case QUEX_TKN_MAGENTA_TEAM:
                        std::cout << "Magenta: " << token.number << std::endl;
                        break;

                    case QUEX_TKN_UNCLASSIFIED:
                        std::cout << "Unclassified: " << token.number << std::endl;
                        break;
                }
            }
            while(token.type_id() != QUEX_TKN_TERMINATION);

            return {};

        }
    }
}