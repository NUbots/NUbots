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

namespace module::extension {

    using component::DirectorTask;
    using component::Provider;
    using component::ProviderGroup;
    using ::extension::behaviour::RunReason;

    Director::RunResult Director::run_tasks(ProviderGroup& our_group,
                                            const TaskList& tasks,
                                            const RunLevel& run_level,
                                            const std::set<std::type_index>& used) {

        // This might happen if only optional tasks are emitted, in that case we successfully run nothing here
        if (tasks.empty()) {
            return RunResult{RunLevel::OK, {}};
        }

        // Create the solution tree for each of the tasks in the pack
        // This Solution::Option would be the option that was executed by the parent of this task
        std::vector<Solution> solutions;
        for (const auto& task : tasks) {
            solutions.push_back(solve_task(task));
        }

        if (run_level >= RunLevel::OK) {
            // Try to find an ok solution for the pack
            auto ok_solutions = find_ok_solutions(solutions, used);

            // Loop through the solutions and add watches for the things we want
            for (int i = 0; i < int(tasks.size()); ++i) {
                auto& task = tasks[i];
                auto& sol  = ok_solutions[i];

                // Both the types we used and types we were blocked by are things we want to watch
                std::set<std::type_index> w = sol.used;
                w.insert(sol.blocking_groups.begin(), sol.blocking_groups.end());

                for (const auto& type : w) {
                    our_group.watch_handles.push_back(groups.at(type).add_watcher(task));
                }
            }

            // Check if any of the solutions are blocked, in which case we can't run
            bool blocked =
                std::any_of(ok_solutions.begin(), ok_solutions.end(), [](const auto& s) { return s.blocked; });

            // If we found an OK solution, run it
            if (!blocked) {

                // Tasks that we have kicked out replacing them with our own tasks
                // We need to reevaluate the group that emitted these tasks when we are done
                TaskList displaced_tasks;

                // Loop through each pair of solutions and task
                for (int i = 0; i < int(tasks.size()); ++i) {
                    auto& sol                    = ok_solutions[i];
                    const auto& needed_providers = sol.requirements;
                    const auto& main_provider    = sol.provider;
                    auto& main_group             = main_provider->group;
                    const auto& new_task         = tasks[i];
                    auto& current_task           = main_group.active_task;

                    // If this is the same task we are already running we skip it
                    if (new_task != current_task) {

                        // If we emitted the last task that went here we can just run and do a replacement
                        // No need to check any lower down providers as they'll already be handled from last time
                        if (current_task != nullptr
                            && providers.at(current_task->requester_id)->type
                                   == providers.at(new_task->requester_id)->type) {
                            run_task_on_provider(new_task, main_provider, RunReason::NEW_TASK);
                        }
                        else {
                            if (current_task != nullptr) {
                                displaced_tasks.push_back(current_task);
                            }

                            // Loop through all but the first needed provider and set their task to this task
                            // This is to reserve those tasks for when our child task executes
                            // We skip this step however if we are a parent of the currently running task
                            // That can happen if we just change providers within the same group (pushed etc)
                            for (int j = 1; j < int(needed_providers.size()); ++j) {
                                // TODO this happens after displacement?
                                // TODO this also influences displacement in that if we displace a parent and a child
                                // then we need to reevaluate them differently since challenge priority doesn't work
                                // properly anymore
                                // TODO probably need to do a displacement pass, followed by a reevaluation pass

                                // TODO if we do this there is no way to clean it up later currently?

                                // const auto& provider = needed_providers[j];
                                // auto& g              = provider->group;

                                // TODO if we are a parent of the currently running task, we can skip this
                                // TODO what if the provider changes from active provider? (it shouldn't?)
                            }

                            // Run this specific task using this specific provider
                            run_task_on_provider(new_task, main_provider, RunReason::NEW_TASK);
                        }
                    }
                }

                // Sort the tasks so we reevaluate them in priority order
                std::sort(displaced_tasks.begin(), displaced_tasks.end(), [this](const auto& a, const auto& b) {
                    return !challenge_priority(a, b);
                });

                for (const auto& t : displaced_tasks) {
                    // Everyone we displaced needs to be reevaluated
                    reevaluate_group(providers.at(t->requester_id)->group);
                }

                // Accumulate all the used types
                std::set<std::type_index> used_types;
                for (const auto& s : ok_solutions) {
                    used_types.insert(s.used.begin(), s.used.end());
                }
                return {RunLevel::OK, used_types};
            }
        }

        // We are able to push others
        if (run_level >= RunLevel::PUSH) {
            // TODO Queue everywhere, setup push providers
        }

        // If we reach here, we have already queued into everything that could possibly change our state
        return RunResult{RunLevel::BLOCKED, {}};
    }

    void Director::run_task_pack(const TaskPack& pack) {

        const auto& provider        = pack.provider;
        const auto& requested_tasks = pack.tasks;
        auto& group                 = provider->group;

        // Check if this Provider is active and allowed to make subtasks
        if (provider != group.active_provider) {
            return;
        }

        // See if a Idle command was emitted
        for (const auto& t : requested_tasks) {
            if (t->type == typeid(::extension::behaviour::Continue)) {
                if (requested_tasks.size() > 1) {
                    log<WARN>("Idle task was emitted with other tasks, the other tasks will be ignored");
                }

                // We don't do anything else on idle
                return;
            }
        }

        // See if a done command was emitted
        for (const auto& t : requested_tasks) {
            if (t->type == typeid(::extension::behaviour::Done)) {
                auto parent_provider = providers.at(group.active_task->requester_id);

                // Check if we are already done, and if so we don't want to pester the parent again
                if (group.done) {
                    // Running Done when already in a Done state shouldn't happen for a root task since it should
                    // already have been removed
                    if (parent_provider->classification == Provider::Classification::ROOT) {
                        log<ERROR>("Done task was emitted twice, this should never happen for a root task");
                    }
                    return;
                }

                // This provider is now in the done state
                group.done = true;
                group.update_data();

                auto& parent_group = parent_provider->group;

                // If it's a root provider, then we just remove the task
                if (parent_provider->classification == Provider::Classification::ROOT) {
                    auto task = parent_group.subtasks.front();
                    parent_group.subtasks.clear();
                    parent_group.watch_handles.clear();
                    remove_task(task);
                }
                else {
                    // TODO(thouliston) check somehow if this provider is equipped to handle done
                    run_task_on_provider(parent_group.active_task,
                                         parent_group.active_provider,
                                         RunReason::SUBTASK_DONE);
                }

                if (requested_tasks.size() > 1) {
                    log<WARN>("Done task was emitted with other tasks, the other tasks will be ignored");
                }

                // We don't do anything else on done
                return;
            }
        }

        // Check if a Wait command was emitted and schedule to run the Provider again
        // Other tasks can run with Wait
        for (const auto& t : requested_tasks) {
            if (t->type == typeid(::extension::behaviour::Wait)) {
                // Schedule the Provider to run again
                // Get the time to wait for
                auto wait_data                 = std::static_pointer_cast<::extension::behaviour::Wait>(t->data);
                std::chrono::nanoseconds delay = std::chrono::nanoseconds(wait_data->time - NUClear::clock::now());

                // If the delay is over, just run the provider
                if (delay <= std::chrono::nanoseconds(0)) {
                    run_task_on_provider(group.active_task, provider, RunReason::SUBTASK_DONE);
                    return;
                }

                // Otherwise, send it to the ChronoController to handle
                // Make a weak pointer to the task so we can check if it still exists when the task is run
                std::weak_ptr<component::DirectorTask> weak_task = t;
                emit(std::make_unique<NUClear::dsl::operation::ChronoTask>(
                    [this, provider, weak_task](const NUClear::clock::time_point&) {
                        // Check if the task still exists
                        auto task = weak_task.lock();
                        if (weak_task) {
                            emit(std::make_unique<WaitDelay>(provider));
                        }
                        // Don't do anything else with this task
                        return false;
                    },
                    NUClear::clock::now() + delay,
                    -1));  // Our ID is -1 as we will remove ourselves
            }
        }

        // If we get here, the provider is not done and we are running new tasks
        group.done = false;
        group.update_data();

        // Remove null data tasks from the list, this allows root tasks to be cleared
        TaskList tasks;
        tasks.reserve(requested_tasks.size());
        for (const auto& t : requested_tasks) {
            if (t->data != nullptr) {
                tasks.push_back(t);
            }
        }

        // We need to check if we dropped our priority for a subtask we already have
        // If so that task might get kicked out by a higher priority task
        // Before we run the new pack we need to see if those tasks would be kicked out
        // We do this by updating the existing tasks priority to the new priority and then reevaluating their group
        // which will kick them out if they are no longer the preferred task or if they are it will do nothing
        TaskList lowered_tasks;
        for (auto& subtask : group.subtasks) {
            // See if we have a matching task
            auto it = std::find_if(tasks.begin(), tasks.end(), [&](const std::shared_ptr<DirectorTask>& t) {
                return subtask->type == t->type;
            });

            // Update the old tasks priority
            if (it != tasks.end() && direct_priority(*it, subtask)) {
                lowered_tasks.push_back(subtask);
                subtask->priority = (*it)->priority;
                subtask->optional = (*it)->optional;
            }
        }
        for (const auto& t : lowered_tasks) {
            // If we were running this task and we lowered its priority, we need to reevaluate
            // Somewhere in the tree down there might be a task that changes
            auto& g = groups.at(t->type);
            if (t == g.active_task) {
                reevaluate_children(g);
            }
        }

        // Clear all the queue handles so their destructors remove us from all of the watchers we were in
        group.watch_handles.clear();

        // This new task pack we are trying to run might be different from the task pack previously ran.
        // There might be some tasks that weren't emitted in the new task pack.
        // These tasks need to be removed from the places they were running.
        // Set them as dying so that they are removed at the end of this function.
        // The dying state stops the task being run again when it shouldn't before it is removed.
        for (const auto& t : group.subtasks) {
            // Search the new tasks for an equivalent task and if we can't find it set it as dying
            auto f = [t](const auto& t2) { return t->type == t2->type; };
            if (std::find_if(tasks.begin(), tasks.end(), f) == tasks.end()) {
                t->dying = true;
            }
        }

        // Once we start running tasks we need to keep track of providers that we are using
        // Future optional tasks might try to run and because priority says that you can replace existing tasks from
        // your current provider they would be allowed (incorrectly) to take over. We can't change this behaviour as
        // this would be correct behaviour if we emitted a new task. We should be able to replace our old tasks.
        std::set<std::type_index> used;

        // Run the first task pack segment as all tasks that are not optional
        auto first_optional = std::find_if(tasks.begin(), tasks.end(), [](const auto& t) { return t->optional; });
        auto result         = run_tasks(group, TaskList(tasks.begin(), first_optional), RunLevel::OK, used);
        used.insert(result.used.begin(), result.used.end());

        // If we are not running normally now and were previously we might still have active tasks that need removing
        if (result.run_level != RunLevel::OK) {
            for (auto& t : group.subtasks) {
                remove_task(t);
            }
        }

        // Run each of the optional tasks but only to the level the main task ran or pushed
        if (result.run_level <= RunLevel::PUSH) {
            for (auto it = first_optional; it != tasks.end(); ++it) {
                // Run each optional task in turn as its own pack
                // but if the main group was pushed, we can at most push in optional
                auto optional_result = run_tasks(group, TaskList({*it}), result.run_level, used);
                used.insert(optional_result.used.begin(), optional_result.used.end());
                if (optional_result.run_level != RunLevel::OK) {
                    // If we can't run this optional task, remove it in case we were previously running it
                    auto original = std::find_if(group.subtasks.begin(), group.subtasks.end(), [&](const auto& t) {
                        return t->type == (*it)->type;
                    });
                    if (original != group.subtasks.end()) {
                        remove_task(*original);
                    }
                }
            }
        }


        // Make a copy of group.subtasks so we can remove tasks from it with updated subtasks
        auto old_subtasks = group.subtasks;
        // Update the group's subtasks to the new subtasks
        group.subtasks = tasks;

        // Remove any tasks that were marked as dying earlier
        // This is at the end, so that if one of our new tasks uses something the old tasks did then we won't run other
        // tasks with lower priority for a single cycle until this one runs
        for (const auto& t : old_subtasks) {
            if (t->dying) {
                remove_task(t);
            }
        }
    }

}  // namespace module::extension
