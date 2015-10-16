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

#ifndef MODULES_RESEARCH_PRESSUREAUTOCLASSIFIER_H
#define MODULES_RESEARCH_PRESSUREAUTOCLASSIFIER_H

#include <nuclear>

#include "messages/vision/LookUpTable.h"

namespace modules {
namespace research {

    class PressureAutoClassifier : public NUClear::Reactor {
    private:
        std::map<messages::vision::Colour, uint> maxSurfaceArea;
        std::map<messages::vision::Colour, uint> maxVolume;
        std::map<messages::vision::Colour, uint> volume;
        std::map<messages::vision::Colour, std::set<uint>> surfaceArea;
        std::map<messages::vision::Colour, uint> zeroPoints;
        std::map<messages::vision::Colour, uint> zeroPointGrowths;
        std::map<messages::vision::Colour, uint> voteGrowths;
        std::map<messages::vision::Colour, uint> maxVotes;
        std::map<uint, uint> votes;

    public:
        /// @brief Called by the powerplant to build and setup the PressureAutoClassifier reactor.
        explicit PressureAutoClassifier(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif
