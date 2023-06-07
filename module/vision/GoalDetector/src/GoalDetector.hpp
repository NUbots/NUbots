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

#ifndef MODULES_VISION_GOALDETECTOR_HPP
#define MODULES_VISION_GOALDETECTOR_HPP

#include <Eigen/Core>
#include <nuclear>

namespace module::vision {

    class GoalDetector : public NUClear::Reactor {
    private:
        struct {
            double confidence_threshold                = 0.0f;
            int cluster_points                         = 0;
            double disagreement_ratio                  = 0.0f;
            Eigen::Vector3d goal_projection_covariance = Eigen::Vector3d::Zero();
            bool use_median                            = false;
            double max_goal_distance                   = 0;
        } config{};

    public:
        /// @brief Called by the powerplant to build and setup the GoalDetector reactor.
        explicit GoalDetector(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::vision


#endif
