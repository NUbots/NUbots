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

#ifndef MODULE_MOTION_WALK_TORSOCONTROLLER_HPP
#define MODULE_MOTION_WALK_TORSOCONTROLLER_HPP

#include <Eigen/Geometry>

namespace module {
    namespace motion {
        namespace walk {

            class TorsoController {
            public:
                Eigen::Affine3d next_torso(const double& time_horizon,
                                           const double& time_left,
                                           const Eigen::Affine3d& Htg,
                                           const Eigen::Affine3d& Ht_tg);

                struct {
                    // Limit the amount that the robot can move in one time step to prevent large sudden movements
                    double max_translation;
                    double max_rotation;
                    // If the distance to the target is less than this threshold, go directly to the target
                    double translation_threshold;
                    // If the distance to the target rotation is less than this threshold, go directly to the target
                    // rotation
                    double rotation_threshold;
                } config;

            private:
                Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Hgo;
            };

        }  // namespace walk
    }      // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_TORSOCONTROLLER_HPP
