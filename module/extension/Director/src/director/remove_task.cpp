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

    void Director::transfer_pushes(const std::shared_ptr<DirectorTask>& from, const std::shared_ptr<DirectorTask>& to) {
        for (auto& [type, g] : groups) {
            if (g.pushing_task == from) {
                g.pushing_task = to;
            }
        }
    }

    void Director::release_pushes(const std::shared_ptr<DirectorTask>& task) {

        // Find and clear every push held by this task before any reevaluation can see half cleared state
        std::vector<std::type_index> released;
        for (auto& [type, g] : groups) {
            if (g.pushing_task == task) {
                g.pushing_task    = nullptr;
                g.pushed_provider = nullptr;
                if (g.active_task != nullptr) {
                    released.push_back(type);
                }
            }
        }

        // Sort so that the released groups with the highest priority tasks react first
        std::stable_sort(released.begin(), released.end(), [this](const auto& a, const auto& b) {
            // If b wins against a, then we swap
            return challenge_priority(groups.at(b).active_task, groups.at(a).active_task);
        });

        // Reevaluating the group lets its queued watchers take over the push, and failing that the group's own
        // task re-runs and reverts to its preferred provider. Each group is rechecked before reevaluating since an
        // earlier reevaluation may have re-pushed, retasked or torn it down.
        for (const auto& type : released) {
            if (groups.contains(type)) {
                auto& g = groups.at(type);
                if (!g.zombie && g.pushing_task == nullptr && g.active_task != nullptr) {
                    reevaluate_group(g);
                }
            }
        }
    }

    void Director::remove_task(const std::shared_ptr<DirectorTask>& task, const bool& release) {

        // If this task is going away for good it can't push anything anymore, release its pushes even if it never
        // got to run itself. A task that is merely stopping execution but staying queued keeps its pushes.
        if (release) {
            release_pushes(task);
        }

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
                        auto lock             = group.update_data(RunReason::STOPPED);
                        powerplant.submit(provider->reaction->get_task());
                    }
                }
                group.active_task     = nullptr;
                group.active_provider = nullptr;

                // Update the group information to reflect that this provider group is no longer running
                group.update_data();

                // If anyone was pushing this group they can't push anymore since we are not active.
                // Release the push before reevaluating so the pusher's re-solve sees consistent state, it will
                // either find another way to push or queue up blocked.
                if (group.pushing_task != nullptr) {
                    auto pusher_task      = group.pushing_task;
                    group.pushing_task    = nullptr;
                    group.pushed_provider = nullptr;

                    // Reevaluate whoever pushed us
                    auto& pusher = providers.at(pusher_task->requester_id)->group;
                    reevaluate_group(pusher);
                }

                auto subtasks = group.subtasks;
                group.subtasks.clear();
                group.watch_handles.clear();

                // Remove any subtasks this group had recursively
                for (const auto& t : subtasks) {
                    remove_task(t);
                }
            }
            // After we have removed all our subtasks we are no longer a zombie, we are just dead
            group.zombie = false;
        }
    }

}  // namespace module::extension
