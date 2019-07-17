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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_GOALDETECTOR_H
#define MODULES_VISION_GOALDETECTOR_H

#include <nuclear>

#include "message/conversion/math_types.h"

namespace module {
namespace vision {

    class GoalDetector : public NUClear::Reactor {
    private:
        struct {
            float confidence_threshold;
            int cluster_points;
            float disagreement_ratio;
            message::conversion::math::fmat3 goal_angular_cov;
            bool use_median;
            bool debug;
        } config;

    public:
        /// @brief Called by the powerplant to build and setup the GoalDetector reactor.
        explicit GoalDetector(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace vision
}  // namespace module


#endif
