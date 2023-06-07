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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_SKILLS_DIRECTWALKCONTROLLER_HPP
#define MODULES_BEHAVIOUR_SKILLS_DIRECTWALKCONTROLLER_HPP

#include <nuclear>

namespace module::behaviour::skills {

    class DirectWalkController : public NUClear::Reactor {
    private:
        /// @brief Subsumption ID key to access motors
        const size_t subsumption_id;

        /// @brief Stores configuration values
        struct Config {
            Config() = default;
            /// @brief DirectWalkController priority in the subsumption system
            double direct_walk_priority = 0.0f;
        } cfg;

        /// @brief Updates the priority of the module by emitting an ActionPriorities message
        /// @param priority The priority used in the ActionPriorities message
        void update_priority(const double& priority);

    public:
        /// @brief Called by the powerplant to build and setup the DirectWalkController reactor.
        explicit DirectWalkController(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::skills

#endif  // MODULES_BEHAVIOUR_SKILLS_DIRECTWALKCONTROLLER_HPP
