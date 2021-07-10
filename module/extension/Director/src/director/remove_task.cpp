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
#include "provider/ProviderGroup.hpp"

namespace module::extension {

    using ::extension::behaviour::commands::DirectorTask;

    void Director::remove_task(const std::shared_ptr<const DirectorTask>& task) {

        // Get the group for this task
        auto& group = groups[task->type];

        // Check if this task is the currently active task this provider is running
        if (task == group.active_task) {

            // Remove this task, we are no longer doing it
            group.active_task = nullptr;

            // Re-evaluate the queue of tasks for this group
            // This may set `group.active_task` to a valid value
            reevaluate_queue(group);

            // If nothing in the queue updated the active task to a new task we are now idle
            // That also means we need to remove any subtasks this group had recursively
            if (group.active_task == nullptr) {
                group.state = provider::ProviderGroup::IDLE;

                // TODO(@TrentHouliston)
                //  if there is a pure leaving provider, run it
                //  maybe I need a new set of DSL keywords so there can be a pure leaving keyword that isn't leaving
                //  what leaving without causing/when providers do are very different from what they do with
                //      when/causing.
                //  maybe the leaving without a causing needs it's own DSL keyword, (might be leaving and current
                //      leaving needs renaming)

                for (const auto& t : group.subtasks) {
                    remove_task(t);
                }
            }
        }
        else {
            // Erase this task from the queued tasks
            group.task_queue.erase(std::remove(group.task_queue.begin(), group.task_queue.end(), task),
                                   group.task_queue.end());
        }
    }

}  // namespace module::extension
