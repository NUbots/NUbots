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

    void remove_task(const std::shared_ptr<const DirectorTask>& task) {

        // Get the group for this task
        auto& group = groups[task->type];

        // Check if this task is the currently active task this provider is running
        if (task == group.active_task) {

            // There are no tasks queued to be executed by this group.
            if (group.queued.empty()) {

                // We are now idle
                group.state = ProviderGroup::IDLE;

                // TODO if there is a pure leaving provider, run it
                // TODO maybe I need a new set of DSL keywords so there can be a pure leaving keyword that isn't leaving
                // TODO what leaving without causing/when providers do are very different from what they do with
                //      when/causing.
                // TODO maybe the leaving without a causing needs it's own DSL keyword, (might be leaving and current
                //      leaving needs renaming)

                // Remove all the subtasks from this provider group recursively
                for (const auto& t : group.subtasks) {
                    remove_task(t);
                }
            }
            else {
                // Sort queued tasks by priority
                std::sort(group.queued_tasks.begin(), group.queued_tasks.end(), [this](const auto& a, const auto& b) {
                    return challenge_priority(a, b);
                });

                // Go through each of the queued tasks to determine if it is the chosen one!
                for (const auto& t : group.queued_tasks) {
                    auto& g = groups[t->type];
                    // Options that could happen
                    //
                    //      The queued task is an optional task and all required tasks of the group g are executable or
                    //      already are executing on their providers. Then we just have to run this specific task.
                    //          run_task(t)
                    //
                    //      This task is required and the queued task's provider group is able to run (all required
                    //      tasks are executable). That means that waiting on this provider was all that was stopping it
                    //      from running. We can now just re-run the entire task pack so it can take the control it
                    //      needs.
                    //          run_task_pack(g.subtasks)
                    //
                    //      The group can't run because it doesn't have enough priority for another required task that
                    //      the task emitted
                    //          Skip this group task, it can't run and is waiting on something else besides us
                    //
                    //      The group can't run because it needs a causing condition that isn't met yet.
                    //      Maybe now that this task is gone, there is a better causing that it has access to.
                    //          TODO need to work out if this groups task pack would change anything if we ran it in
                    //          regards to having access to this provider groups providers (is there an entering
                    //          provider?)
                    //
                    //      TODO are there any other things that could happen?
                    //
                    //      TODO if none of the tasks that are in the queue can actually run, we just treat this the
                    //      same way as if this list was empty and delete all the subtasks and run empty leaving if one
                    //      exists
                }
            }
        }
        else {
            // Erase this task from the queued tasks
            std::erase(std::remove(group.queued_tasks.begin(), group.queued_tasks.end(), task),
                       group.queued_tasks.end());
        }
    }

}  // namespace module::extension
