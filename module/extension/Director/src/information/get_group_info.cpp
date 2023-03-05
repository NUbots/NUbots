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

    using ::extension::behaviour::GroupInfo;

    GroupInfo Director::_get_group_info(const uint64_t& reaction_id,
                                        const std::type_index& type,
                                        const std::type_index& root_type) {
        std::lock_guard<std::recursive_mutex> lock(director_mutex);

        uint64_t r_id = reaction_id;

        // If it is a root provider, get the root provider id
        if (!providers.contains(reaction_id)) {
            r_id = get_root_provider(root_type)->id;
        }

        if (groups.contains(type)) {
            // Check if the task is active
            if (groups.at(type).active_task != nullptr) {
                // Check if the task is being run by this reaction
                if (groups.at(type).active_task->requester_id == r_id) {
                    return GroupInfo{GroupInfo::RunState::RUNNING, groups.at(type).done};
                }
            }

            // Check if this task is in the reaction's task list
            // If it is, then it is waiting to be run
            if (providers.contains(r_id)) {
                for (const auto& task : providers.at(r_id)->group.subtasks) {
                    if (task->type == type) {
                        return GroupInfo{GroupInfo::RunState::QUEUED, groups.at(type).done};
                    }
                }
            }

            // Not running or queued, so it must not have been requested yet
            return GroupInfo{GroupInfo::RunState::NO_TASK, groups.at(type).done};
        }
        else {
            throw std::runtime_error("No group with the requested type");
        }
    }

}  // namespace module::extension
