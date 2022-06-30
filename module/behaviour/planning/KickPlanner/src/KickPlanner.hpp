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

#ifndef MODULES_BEHAVIOUR_PLANNING_KICKPLANNER_HPP
#define MODULES_BEHAVIOUR_PLANNING_KICKPLANNER_HPP

#include "message/input/Sensors.hpp"
#include "message/motion/KickCommand.hpp"
namespace module::behaviour::planning {

    class KickPlanner : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and setup the KickPlanner reactor.
        explicit KickPlanner(std::unique_ptr<NUClear::Environment> environment);

    private:
        [[nodiscard]] bool kickValid(const Eigen::Vector3d& ballPos) const;
        bool forcePlaying = false;
        message::motion::KickPlannerConfig cfg{};
        NUClear::clock::time_point ballLastSeen{std::chrono::seconds(0)};
        NUClear::clock::time_point lastTimeValid{NUClear::clock::now()};
    };
}  // namespace module::behaviour::planning


#endif
