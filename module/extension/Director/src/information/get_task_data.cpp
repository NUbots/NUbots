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

    std::shared_ptr<void> Director::_get_task_data(const uint64_t& reaction_id) {
        // How did we get here?
        if (providers.count(reaction_id) == 0) {
            return nullptr;
        }

        // Get the provider and group for the reaction
        auto provider = providers.at(reaction_id);
        auto group    = groups[provider->type];

        // Only the active provider is allowed to have data
        if (provider->active) {
            return nullptr;
        }

        // If the task is of the wrong type we can't return it
        if (group.active_task->type != provider->type) {
            return nullptr;
        }

        // Return the data
        return group.active_task->data;
    }

}  // namespace module::extension
