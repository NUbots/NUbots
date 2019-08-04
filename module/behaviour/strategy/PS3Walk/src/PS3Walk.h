/*
 * This file is part of PS3Walk.
 *
 * PS3Walk is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PS3Walk is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PS3Walk.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_STRATEGY_PS3WALK_H
#define MODULES_BEHAVIOUR_STRATEGY_PS3WALK_H


#include <Eigen/Core>
#include <nuclear>
#include <string>
#include <vector>

namespace module {
namespace behaviour {
    namespace strategy {

        class PS3Walk : public NUClear::Reactor {
        public:
            /// @brief Called by the powerplant to build and setup the PS3Walk reactor.
            explicit PS3Walk(std::unique_ptr<NUClear::Environment> environment);

        private:
            size_t id;

            Eigen::Vector3d walkCommandLimits = Eigen::Vector3d::Zero();
            Eigen::Vector3d walkCommand       = Eigen::Vector3d::Zero();
            Eigen::Vector3d prevWalkCommand   = Eigen::Vector3d::Zero();

            Eigen::Vector2d headCommandLimits = Eigen::Vector2d::Zero();
            Eigen::Vector2d headCommand       = Eigen::Vector2d::Zero();
            Eigen::Vector2d prevHeadCommand   = Eigen::Vector2d::Zero();

            double walk_command_threshold = 0.1;
            double head_command_threshold = 0.1;

            bool moving     = false;
            bool headLocked = false;

            std::vector<std::string> actions;
        };
    }  // namespace strategy
}  // namespace behaviour
}  // namespace module


#endif
