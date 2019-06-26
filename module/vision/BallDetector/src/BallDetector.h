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

#ifndef MODULES_VISION_BALLDETECTOR_H
#define MODULES_VISION_BALLDETECTOR_H

#include <Eigen/Core>
#include <nuclear>

#include "message/conversion/math_types.h"

namespace module {
namespace vision {

    class BallDetector : public NUClear::Reactor {
    private:
        struct {
            float confidence_threshold;
            int cluster_points;
            float minimum_ball_distance;
            float distance_disagreement;
            float maximum_deviation;
            message::conversion::math::fmat3 ball_angular_cov;
            bool debug;
        } config;

    public:
        /// @brief Called by the powerplant to build and setup the BallDetector reactor.
        explicit BallDetector(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace vision
}  // namespace module


#endif
