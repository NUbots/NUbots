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

#ifndef MODULES_VISION_QUEXCLASSIFIER_H
#define MODULES_VISION_QUEXCLASSIFIER_H

#include <vector>

#define QUEX_SETTING_BUFFER_MIN_FALLBACK_N 0
#define QUEX_OPTION_ASSERTS_DISABLED
#define QUEX_OPTION_COMPUTED_GOTOS
#define QUEX_OPTION_TERMINATION_ZERO_DISABLED
#include "Lexer.hpp"
#include "messages/input/Image.h"
#include "messages/vision/LookUpTable.h"
#include "messages/vision/ClassifiedImage.h"

namespace modules {
    namespace vision {
        class QuexClassifier {
        private:
            static constexpr size_t BUFFER_SIZE = 2000;
            uint8_t buffer[BUFFER_SIZE]; // This should be big enough for now
            quex::Lexer lexer;
            uint32_t& tknNumber;

        public:
            QuexClassifier();

            std::vector<messages::vision::ClassifiedImage<messages::vision::ObjectClass>::Segment> classify(const messages::input::Image& image, const messages::vision::LookUpTable& lut, const arma::ivec2& start, const arma::ivec2& end, const uint& stratification = 1);
        };
    }
}

#endif