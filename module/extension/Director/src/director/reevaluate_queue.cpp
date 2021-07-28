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

#include "Director.hpp"

namespace module::extension {

    void Director::reevaluate_queue(provider::ProviderGroup& group) {

        // Sort queued tasks by priority
        std::sort(group.task_queue.begin(), group.task_queue.end(), [this](const auto& a, const auto& b) {
            return challenge_priority(a, b);
        });

        // Go through each of the queued tasks and try to re-run them and their siblings
        for (const auto& t : group.task_queue) {

            // This is a root level task, therefore it is its own pack
            if (providers.count(t->requester_id) == 0) {
                run_task_pack({t});
            }
            else {
                // Find the group that requested this task and rerun its task pack
                auto p  = providers[t->requester_id];
                auto& g = groups[p->type];
                run_task_pack(g.subtasks);
            }

            // If running this task pack claimed the active task we can stop looking
            if (group.active_task == t) {
                break;
            }
        }
    }

}  // namespace module::extension
