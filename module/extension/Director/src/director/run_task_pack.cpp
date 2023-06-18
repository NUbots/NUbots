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

    using component::DirectorTask;
    using component::Provider;
    using component::ProviderGroup;
    using ::extension::behaviour::RunInfo;

    Director::RunLevel Director::run_tasks(ProviderGroup& our_group, const TaskList& tasks, const RunLevel& run_level) {

        // This might happen if only optional tasks are emitted, in that case we successfully run nothing here
        if (tasks.empty()) {
            return RunLevel::OK;
        }

        // Create the solution tree for each of the tasks in the pack
        // This Solution::Option would be the option that was executed by the parent of this task
        std::vector<Solution> solutions;
        for (const auto& task : tasks) {
            solutions.push_back(solve_task(task));
        }

        if (run_level >= RunLevel::OK) {
            // Try to find an ok solution for the pack
            auto ok_solutions = find_ok_solutions(solutions);

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
                            run_task_on_provider(new_task, main_provider, RunInfo::RunReason::NEW_TASK);
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
                            run_task_on_provider(new_task, main_provider, RunInfo::RunReason::NEW_TASK);
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

                return RunLevel::OK;
            }
        }

        // We are able to push others
        if (run_level >= RunLevel::PUSH) {
            // TODO Queue everywhere, setup push providers
        }

        // If we reach here, we have already queued into everything that could possibly change our state
        return RunLevel::BLOCKED;
    }

    void Director::run_task_pack(const TaskPack& pack) {
        const auto& provider = pack.first;
        auto& group          = provider->group;


        // Print the task pack
        log<NUClear::INFO>("Running task pack for provider ",
                           provider->type.name(),
                           " with ",
                           pack.second.size(),
                           " tasks");
        for (const auto& t : pack.second) {
            log<NUClear::INFO>("  ", t->type.name());
        }

        // Print groups and their active task, if they have one
        for (const auto& g : groups) {
            if (g.second.active_task != nullptr) {
                log<NUClear::INFO>("    ", g.first.name(), ": ", g.second.active_task->type.name());
            }
            else {
                log<NUClear::INFO>("    ", g.first.name(), " : no active task");
            }
        }

        // Check if this Provider is active and allowed to make subtasks
        if (provider != group.active_provider) {
            return;
        }

        // See if a Idle command was emitted
        for (const auto& t : pack.second) {
            if (t->type == typeid(::extension::behaviour::Idle)) {

                if (pack.second.size() > 1) {
                    log<NUClear::WARN>("Idle task was emitted with other tasks, the other tasks will be ignored");
                }

                // We don't do anything else on idle
                return;
            }
        }

        // See if a done command was emitted
        for (const auto& t : pack.second) {
            if (t->type == typeid(::extension::behaviour::Done)) {

                // This provider is now in the done state
                provider->group.done = true;

                auto parent_provider = providers.at(group.active_task->requester_id);
                auto& parent_group   = parent_provider->group;

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
                                         RunInfo::RunReason::SUBTASK_DONE);
                }

                if (pack.second.size() > 1) {
                    log<NUClear::WARN>("Done task was emitted with other tasks, the other tasks will be ignored");
                }

                // We don't do anything else on done
                return;
            }
        }

        // If we get here, the provider is not done and we are running new tasks
        provider->group.done = false;

        // Remove null data tasks from the list, this allows root tasks to be cleared
        TaskList tasks;
        tasks.reserve(pack.second.size());
        for (const auto& t : pack.second) {
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

        // Run the first task pack segment as all tasks that are not optional
        auto first_optional = std::find_if(tasks.begin(), tasks.end(), [](const auto& t) { return t->optional; });
        auto run_level      = run_tasks(group, TaskList(tasks.begin(), first_optional), RunLevel::OK);

        // If we are not running normally now and were previously we might still have active tasks that need removing
        if (run_level != RunLevel::OK) {
            for (auto& t : group.subtasks) {
                remove_task(t);
            }
        }

        // Run each of the optional tasks but only to the level the main task ran or pushed
        if (run_level <= RunLevel::PUSH) {
            for (auto it = first_optional; it != tasks.end(); ++it) {
                // Run each optional task in turn as its own pack
                // but if the main group was pushed, we can at most push in optional
                if (run_tasks(group, TaskList({*it}), run_level) != RunLevel::OK) {
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
