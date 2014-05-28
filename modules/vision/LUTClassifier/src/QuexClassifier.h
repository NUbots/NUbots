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

#include <map>

#include "Lexer.hpp"
#include "messages/input/Image.h"
#include "messages/vision/LookUpTable.h"
#include "messages/vision/ClassifiedImage.h"

namespace modules {
	namespace vision {
		class QuexClassifier {
		private:
			std::vector<uint8_t> buffer;
			quex::Lexer lexer;

			void ensureCapacity();

		public:
			std::multimap<messages::vision::ObjectClass, messages::vision::ClassifiedImage<messages::vision::ObjectClass>::Segment> classify(const messages::input::Image& image, const messages::vision::LookUpTable& lut, const arma::vec2& start, const arma::vec2& end);
		};
	}
}

#endif