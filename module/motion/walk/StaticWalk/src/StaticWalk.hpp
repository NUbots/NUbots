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

#ifndef MODULE_MOTION_WALK_STATICWALK_HPP
#define MODULE_MOTION_WALK_STATICWALK_HPP

#include <Eigen/Geometry>
#include <nuclear>

#include "message/motion/KinematicsModel.hpp"

namespace module {
    namespace motion {
        namespace walk {

            class StaticWalk : public NUClear::Reactor {

            private:
                // Transform from ground to swing foot
                Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Hwg;
                // The states that the robots enters to complete the steps
                enum State { INITIAL, LEFT_LEAN, RIGHT_STEP, RIGHT_LEAN, LEFT_STEP, STOP } state;
                // The time each phase takes to complete
                NUClear::clock::duration phase_time;
                // The time at the start of the current phase
                NUClear::clock::time_point start_phase;
                // Model of the robot
                message::motion::KinematicsModel model;
                // The height of the robots torso (m)
                double torso_height;
                // The width of the robot's stance as it walks (m)
                double stance_width;
                bool start_right_lean;
                double y_offset;
                double x_offset;
                double time;
                double rotation_limit;
                size_t subsumptionId;
                Eigen::Vector3d walkCommand = Eigen::Vector3d::Zero();

                // Reaction handle for the main update loop, disabling when not moving will save unnecessary CPU
                ReactionHandle updateHandle{};

                // Returns ground to torso target for specified lean
                Eigen::Affine3d getLeanTarget(const Eigen::Vector3d& rCTt);

                // Returns ground to foot target for specified step
                Eigen::Affine3d getFootTarget();

            public:
                /// @brief Called by the powerplant to build and setup the StaticWalk reactor.
                explicit StaticWalk(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // namespace walk
    }      // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_STATICWALK_HPP
