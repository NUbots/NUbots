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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#include "Director.hpp"

namespace module::extension {

    using ::extension::behaviour::RunInfo;

    RunInfo Director::_get_run_info(const uint64_t& reaction_id) {
        std::lock_guard<std::recursive_mutex> lock(director_mutex);

        if (providers.contains(reaction_id)) {
            return RunInfo{current_run_reason, providers.at(reaction_id)->group.done};
        }

        log<NUClear::ERROR>("Getting run information for a reaction that isn't a Provider.");
        return RunInfo{current_run_reason, false};
    }

}  // namespace module::extension
