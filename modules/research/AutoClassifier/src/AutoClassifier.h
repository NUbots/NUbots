/*
 * This file is part of NUbots Codebase.
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

#ifndef MODULES_RESEARCH_AUTOCLASSIFIER_H
#define MODULES_RESEARCH_AUTOCLASSIFIER_H

#include <nuclear>
#include "messages/vision/LookUpTable.h"
#include "messages/vision/proto/LookUpTable.pb.h"
#include "messages/input/Image.h"

namespace modules {
namespace research {

    class AutoClassifier : public NUClear::Reactor {
    public:
        struct ColourData {
            bool enabled;
            double range;
            std::vector<messages::input::Image::Pixel> pixels;
        };
        static constexpr const char* CONFIGURATION_PATH = "AutoClassifier.yaml";
        /// @brief Called by the powerplant to build and setup the AutoClassifier reactor.
        explicit AutoClassifier(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::unique_ptr<messages::vision::LookUpTable> reference;

        ColourData orangeData;
        ColourData yellowData;
        ColourData greenData;
        ColourData whiteData;

        ReactionHandle ballClassifier;
        ReactionHandle goalClassifier;
        ReactionHandle fieldClassifier;

        void cacheColours(const messages::vision::LookUpTable& lut);
        void classifyNear(
            const uint x,
            const uint y,
            const messages::input::Image& image,
            messages::vision::LookUpTable& lut,
            const std::vector<messages::input::Image::Pixel>& pixels,
            const messages::vision::Colour& colour,
            const double rangeSqr,
            messages::vision::proto::LookUpTableDiff& tableDiff
        );
    };

}
}


#endif