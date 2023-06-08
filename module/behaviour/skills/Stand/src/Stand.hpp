/*i
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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_REFLEX_STAND_HPP
#define MODULES_BEHAVIOUR_REFLEX_STAND_HPP

#include <nuclear>

namespace module::behaviour::skills {

    /**
     * Executes stand script
     *
     * @author Josiah Walker
     */
    class Stand : public NUClear::Reactor {
    private:
        /// @brief The id registered in the subsumption system for this module
        const size_t subsumption_id;

        /// @brief Stores configuration values
        struct Config {
            Config() = default;
            /// @brief Stand priority in the subsumption system
            double stand_priority = 0.0;
        } cfg;

    public:
        explicit Stand(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::behaviour::skills

#endif  // MODULES_BEHAVIOUR_REFLEX_STAND_HPP
