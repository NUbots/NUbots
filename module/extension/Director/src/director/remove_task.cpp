/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Director.hpp"
#include "component/ProviderGroup.hpp"

namespace module::extension {

    using component::DirectorTask;
    using component::Provider;
    using ::extension::behaviour::RunReason;

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
                        auto lock = group.set_data(RunReason::STOPPED, original_task->data, group.get_group_info());
                        powerplant.submit(provider->reaction->get_task(true));
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
