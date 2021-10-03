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

#include <algorithm>
#include <tuple>
#include <vector>

#include "Director.hpp"

#include "extension/Behaviour.hpp"

namespace module::extension {

    using ::extension::behaviour::commands::DirectorTask;

    // Scope this struct to just this translation unit
    namespace {
        struct TaskPriority {
            TaskPriority(const uint64_t& id_, const int& priority_, const bool& optional_)
                : id(id_), priority(priority_), optional(optional_) {}

            uint64_t id;
            int priority;
            bool optional;
        };
    }  // namespace

    bool Director::challenge_priority(const std::shared_ptr<const DirectorTask>& incumbent,
                                      const std::shared_ptr<const DirectorTask>& challenger) {

        // If there is no incumbent the challenger wins by default
        if (incumbent == nullptr) {
            return true;
        }

        // When a provider emits a new updated task we should always swap to the new task regardless of priority
        // In this case though, we also need to compare to every other task in the queue but this will be done elsewhere
        if (incumbent->requester_id == challenger->requester_id) {
            return true;
        }

        // Function to get the priorities of the ancestors of this task
        auto get_ancestor_priorities = [this](const std::shared_ptr<const DirectorTask>& task) {
            // We are our first ancestor
            std::vector<TaskPriority> ancestors;
            ancestors.emplace_back(task->requester_id, task->priority, task->optional);

            // Loop up through the providers until we reach a point where a task was emitted by a non provider
            for (auto t = task; providers.count(t->requester_id) != 0;) {

                // Get the provider that emitted this task, and from that the provider group
                auto provider = providers[t->requester_id];
                auto& group   = groups[provider->type];

                // If there is no active task something has gone wrong with the algorithm
                if (group.active_task == nullptr) {
                    throw std::runtime_error("Task has broken parentage");
                }

                // Add this new parent we found and set t so we can loop up further
                t = group.active_task;
                ancestors.emplace_back(t->requester_id, t->priority, t->optional);
            }
            return ancestors;
        };

        // Get the ancestor priorities of the two tasks
        auto i_p = get_ancestor_priorities(incumbent);
        auto c_p = get_ancestor_priorities(challenger);

        // Remove all of the common ancestors
        while (i_p.back().id == c_p.back().id) {
            i_p.pop_back();
            c_p.pop_back();
        }

        // Work out if there are any optionals in either of the tasks parentage
        const bool i_o = std::any_of(i_p.begin(), i_p.end(), [](const auto& v) { return v.optional; });
        const bool c_o = std::any_of(c_p.begin(), c_p.end(), [](const auto& v) { return v.optional; });

        // If both or neither are optional then we compare at the point where they were siblings.
        // If both are optional then we would rather that whichever ancestor had higher priority have its optional tasks
        if (i_o == c_o) {
            return i_p.back().priority < c_p.back().priority;
        }

        // If we got here, then the optional status of the two branches are not equal
        // In this case we want to return if incumbent is optional but not the challenger
        // If incumbent is optional this will return true, if it's not this will return false
        return i_o;
    }

}  // namespace module::extension
