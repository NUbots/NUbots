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
#include "component/ProviderGroup.hpp"

namespace module::extension {

    using component::DirectorTask;
    using component::Provider;
    using ::extension::behaviour::RunInfo;

    void Director::remove_task(const std::shared_ptr<DirectorTask>& task) {

        // Get the group for this task
        auto& group = groups.at(task->type);

        // Store the original task so we can use it in stops later if needed
        auto original_task = group.active_task;

        // Check if this task is the currently active task this provider is running
        if (task == group.active_task) {

            // Remove this task, we are no longer doing it
            group.active_task = nullptr;
            group.done        = false;

            // We are now a zombie, we are dead but we are still in the tree
            group.zombie = true;

            // Re-evaluate the group since things may now have changed
            // This may set `group.active_task` to a valid value if a new task is picked up
            reevaluate_group(group);

            // If nothing in the queue updated the active task to a new task we are now idle
            // That also means we need to remove any subtasks this group had recursively
            if (group.active_task == nullptr) {


                // Run the Stop reactions for this provider group since it is no longer running
                // First we restore the original task so we have data for the stop reaction
                group.active_task = original_task;
                for (auto& provider : group.providers) {
                    if (provider->classification == Provider::Classification::STOP) {
                        group.active_provider = provider;
                        auto lock             = hold_run_reason(RunInfo::RunReason::STOPPED);
                        auto task             = provider->reaction->get_task();
                        if (task) {
                            task->run(std::move(task));
                        }
                    }
                }
                group.active_task     = nullptr;
                group.active_provider = nullptr;

                // If anyone was pushing this group they can't push anymore since we are not active
                if (group.pushing_task != nullptr) {
                    // Reevaluate whoever pushed us
                    auto pusher = providers.at(group.pushing_task->requester_id)->group;
                    reevaluate_group(pusher);
                }

                // Remove any subtasks this group had recursively
                for (const auto& t : group.subtasks) {
                    remove_task(t);
                }


                // We now have no subtasks
                group.subtasks.clear();
            }
            // After we have removed all our subtasks we are no longer a zombie, we are just dead
            group.zombie = false;
        }
    }

}  // namespace module::extension
