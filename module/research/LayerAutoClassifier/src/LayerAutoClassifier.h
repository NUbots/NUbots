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

#ifndef MODULES_RESEARCH_LAYERAUTOCLASSIFIER_H
#define MODULES_RESEARCH_LAYERAUTOCLASSIFIER_H

#include <nuclear>

#include "message/vision/LookUpTable.h"

namespace module {
namespace research {

    class LayerAutoClassifier : public NUClear::Reactor {
    private:
        std::map<message::vision::Colour, uint> maxSurfaceArea;
        std::map<message::vision::Colour, uint> maxVolume;
        std::map<message::vision::Colour, uint> volume;
        std::map<message::vision::Colour, std::set<uint>> surfaceArea;

    public:
        /// @brief Called by the powerplant to build and setup the LayerAutoClassifier reactor.
        explicit LayerAutoClassifier(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif