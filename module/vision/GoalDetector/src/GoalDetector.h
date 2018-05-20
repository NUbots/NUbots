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

#include <armadillo>
#include <nuclear>

namespace module {
namespace vision {

    class GoalDetector : public NUClear::Reactor {
    private:
        unsigned int MINIMUM_POINTS_FOR_CONSENSUS;
        unsigned int MAXIMUM_ITERATIONS_PER_FITTING;
        unsigned int MAXIMUM_FITTED_MODELS;
        double CONSENSUS_ERROR_THRESHOLD;

        double MAXIMUM_ASPECT_RATIO;
        double MINIMUM_ASPECT_RATIO;
        double VISUAL_HORIZON_BUFFER;
        double MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE;
        double MAXIMUM_ANGLE_BETWEEN_SIDES;
        double MAXIMUM_VERTICAL_GOAL_PERSPECTIVE_ANGLE;
        arma::running_stat<double> stats;

        unsigned int MEASUREMENT_LIMITS_LEFT;
        unsigned int MEASUREMENT_LIMITS_RIGHT;
        unsigned int MEASUREMENT_LIMITS_TOP;
        unsigned int MEASUREMENT_LIMITS_BASE;

        double ANGULAR_WIDTH_DISAGREEMENT_THRESHOLD_VERTICAL;
        double ANGULAR_WIDTH_DISAGREEMENT_THRESHOLD_HORIZONTAL;

        arma::vec3 VECTOR3_COVARIANCE;
        arma::vec2 ANGLE_COVARIANCE;

        bool DEBUG_GOAL_THROWOUTS;
        bool DEBUG_GOAL_RANSAC;

    public:
        /// @brief Called by the powerplant to build and setup the GoalDetector reactor.
        explicit GoalDetector(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace vision
}  // namespace module


#endif
