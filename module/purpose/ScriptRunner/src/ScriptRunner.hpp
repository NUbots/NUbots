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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_PURPOSE_SCRIPTRUNNER_HPP
#define MODULE_PURPOSE_SCRIPTRUNNER_HPP

#include <nuclear>
#include <vector>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    /**
     * Executes a series of scripts provided by the command line in order.
     *
     * @author Trent Houliston
     */
    class ScriptRunner : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief The scripts to be executed
        std::vector<std::string> scripts{};

    public:
        explicit ScriptRunner(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::purpose

#endif  // MODULE_PURPOSE_SCRIPTRUNNER_HPP
