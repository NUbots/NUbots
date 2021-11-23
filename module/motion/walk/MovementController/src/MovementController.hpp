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

#ifndef MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H
#define MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H

#include <nuclear>

#include "FootController.hpp"
#include "TorsoController.hpp"

namespace module {
    namespace motion {
        namespace walk {
            class MovementController : public NUClear::Reactor {

            public:
                /// @brief Called by the powerplant to build and setup the MovementController reactor.
                explicit MovementController(std::unique_ptr<NUClear::Environment> environment);

            private:
                // Controller for the swing foot
                FootController foot_controller;
                // Controller for the support foot to move the torso
                TorsoController torso_controller;
                struct {
                    double time_horizon;
                    double support_gain;
                    double swing_gain;
                    double swing_lean_gain;
                } config;
            };
        }  // namespace walk
    }      // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H
