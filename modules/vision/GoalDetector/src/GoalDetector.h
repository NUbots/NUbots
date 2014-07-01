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

#ifndef MODULES_VISION_GOALDETECTOR_H
#define MODULES_VISION_GOALDETECTOR_H

#include <nuclear>

#include "utility/math/ransac/ransac.h"

#include "utility/math/geometry/Quad.h"
#include "utility/math/geometry/LSFittedLine.h"

namespace modules {
namespace vision {

    class GoalDetector : public NUClear::Reactor {
    private:
    	unsigned int MINIMUM_POINTS;                            // Minimum points needed to make a line (Min pts to line essentially)
        unsigned int MAX_ITERATIONS_PER_FITTING;                // Number of iterations per fitting attempt
        unsigned int MAX_FITTING_ATTEMPTS;                      // Hard limit on number of fitting attempts
        double CONSENSUS_THRESHOLD;                             // Threshold determining what constitutes a good fit (Consensus margin)
        utility::math::ransac::RansacSelectionMethod SELECTION_METHOD;

    public:

        static constexpr const char* CONFIGURATION_PATH = "GoalDetector.yaml";

        /// @brief Called by the powerplant to build and setup the GoalDetector reactor.
        explicit GoalDetector(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif