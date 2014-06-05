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

        std::vector<ClassifiedImage<ObjectClass>::Segment> QuexClassifier::classify(const Image& image, const LookUpTable& lut, const arma::uvec2& start, const arma::uvec2& end) {

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
            Token lexeme;

            // Our output, the previously inserted token is also held to perform linking
            std::vector<ClassifiedImage<ObjectClass>::Segment> output;

            // Our vector of position
            arma::uvec2 position = start;

            // A reference to the relevant movement direction
            uint& movement = start[1] == end[1] ? position[0] : position[1];

            do {
                lexer.token_p_switch(&lexeme);
                lexer.receive();

                // Update our position
                arma::uvec2 s = position;
                movement += lexeme.number;

                switch(lexeme.type_id()) {
                    case QUEX_TKN_FIELD:
                        output.push_back({ObjectClass::FIELD, s, position, nullptr, nullptr});
                        break;

                    case QUEX_TKN_BALL:
                        output.push_back({ObjectClass::BALL, s, position, nullptr, nullptr});
                        break;

                    case QUEX_TKN_GOAL:
                        output.push_back({ObjectClass::GOAL, s, position, nullptr, nullptr});
                        break;

                    case QUEX_TKN_LINE:
                        output.push_back({ObjectClass::LINE, s, position, nullptr, nullptr});
                        break;

                    case QUEX_TKN_CYAN_TEAM:
                        output.push_back({ObjectClass::CYAN_TEAM, s, position, nullptr, nullptr});
                        break;

                    case QUEX_TKN_MAGENTA_TEAM:
                        output.push_back({ObjectClass::MAGENTA_TEAM, s, position, nullptr, nullptr});
                        break;

                    case QUEX_TKN_UNCLASSIFIED:
                        output.push_back({ObjectClass::UNKNOWN, s, position, nullptr, nullptr});
                        break;
                }
            }
            while(lexeme.type_id() != QUEX_TKN_TERMINATION);

            return output;

        }
    }
}