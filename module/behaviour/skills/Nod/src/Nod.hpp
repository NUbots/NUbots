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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_BEHAVIOUR_SKILLS_NOD_HPP
#define MODULE_BEHAVIOUR_SKILLS_NOD_HPP

#include <nuclear>

namespace module::behaviour::skills {

    class Nod : public NUClear::Reactor {
    private:
        const size_t subsumption_id{size_t(this) * size_t(this) - size_t(this)};

        /// @brief Stores configuration values
        struct Config {
            Config() = default;
            /// @brief Nod priority in the subsumption system
            double nod_priority = 0.0;
        } cfg;

        /// @brief Bool to indicate if the robot should nod yes, else it nods no
        bool nod_yes = false;

        void updatePriority(const double& priority);

    public:
        /// @brief Called by the powerplant to build and setup the KickScript reactor.
        explicit Nod(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::skills


#endif  // MODULE_BEHAVIOUR_SKILLS_NOD_HPP
