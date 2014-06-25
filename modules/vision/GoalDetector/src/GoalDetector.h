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

        double ANGLE_MARGIN;                                    // Used for filtering out goal posts which are on too much of a lean.
        double CONSENSUS_THRESHOLD;                             // Threshold dtermining what constitutes a good fit (Consensus margin)

        utility::math::ransac::RansacSelectionMethod SELECTION_METHOD;
//          Horizon& m_kinematicsHorizon;

        double RANSAC_MATCHING_TOLERANCE;

        int MIN_GOAL_SEPARATION;
        float GOAL_HEIGHT_TO_WIDTH_RATIO_MIN;

        // Constants for construction of a Goal object.
        bool THROWOUT_SHORT_GOALS;
        bool THROWOUT_NARROW_GOALS;
        bool THROWOUT_ON_ABOVE_KIN_HOR_GOALS;
        bool THROWOUT_DISTANT_GOALS;
        float MAX_GOAL_DISTANCE;
        int MIN_GOAL_HEIGHT;
        int MIN_GOAL_WIDTH;
        float GOAL_WIDTH;
        //DISTANCE_METHOD GOAL_DISTANCE_METHOD;
        int EDGE_OF_SCREEN_MARGIN;
        float D2P_ADAPTIVE_THRESHOLD;

        std::vector<utility::math::geometry::Quad> buildQuadsFromLines(const std::vector<utility::math::geometry::LSFittedLine>& startLines,
                                                  const std::vector<utility::math::geometry::LSFittedLine>& endLines, double tolerance);

        void removeInvalid(std::vector<utility::math::geometry::Quad>& posts);

        void mergeClose(std::vector<utility::math::geometry::Quad>& posts, double widthMultipleToMerge);

        unsigned int getClosestUntriedLine(const utility::math::geometry::LSFittedLine& start,
                                                        const std::vector<utility::math::geometry::LSFittedLine>& endLines,
                                                        std::vector<bool>& tried);
    public:

        static constexpr const char* CONFIGURATION_PATH = "GoalDetector.yaml";

        /// @brief Called by the powerplant to build and setup the GoalDetector reactor.
        explicit GoalDetector(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif