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

#ifndef MODULES_VISION_BALLDETECTOR_HPP
#define MODULES_VISION_BALLDETECTOR_HPP

#include <Eigen/Core>
#include <nuclear>

namespace module::vision {

    class BallDetector : public NUClear::Reactor {
    private:
        struct {
            float confidence_threshold       = 0;
            int cluster_points               = 0;
            float minimum_ball_distance      = 0;
            float distance_disagreement      = 0;
            float maximum_deviation          = 0;
            Eigen::Vector3f ball_angular_cov = Eigen::Vector3f::Zero();
            bool debug                       = false;
        } config;

    public:
        /// @brief Called by the powerplant to build and setup the BallDetector reactor.
        explicit BallDetector(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::vision


#endif
