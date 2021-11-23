/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_MOTION_WALK_FOOTCONTROLLER_HPP
#define MODULE_MOTION_WALK_FOOTCONTROLLER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>


namespace module {
    namespace motion {
        namespace walk {

            class FootController {
            public:
                // Calculates the next swing position
                Eigen::Affine3d next_swing(const double& time_horizon,
                                           const double& time_left,
                                           const Eigen::Affine3d& Hwg,
                                           const Eigen::Affine3d& Hw_tg);

                // Stores config variables for the foot controller
                struct {
                    // Vector field variables
                    double step_height;
                    double scaling_factor;
                    double integral_steps;
                    // Limit the amount that the robot can move in one time step to prevent large sudden movements
                    double max_translation;
                    double max_rotation;
                    // If the distance to the target is less than this threshold, go directly to the target
                    double translation_threshold;
                    // If the distance to the target rotation is less than this threshold, go directly to the target
                    // rotation
                    double rotation_threshold;
                } config;
            };

        }  // namespace walk
    }      // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_FOOTCONTROLLER_HPP
