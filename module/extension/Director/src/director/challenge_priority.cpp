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

    using ::extension::behaviour::commands::BehaviourTask;
    using provider::Provider;

    // Scope this struct to just this translation unit
    namespace {
        struct TaskPriority {
            TaskPriority(const std::type_index& requester_, const int& priority_, const bool& optional_)
                : requester(requester_), priority(priority_), optional(optional_) {}

            std::type_index requester;
            int priority;
            bool optional;
        };
    }  // namespace

    bool Director::challenge_priority(const std::shared_ptr<BehaviourTask>& incumbent,
                                      const std::shared_ptr<BehaviourTask>& challenger) {

        // If there is no incumbent the challenger wins by default
        if (incumbent == nullptr) {
            return true;
        }
        if (challenger == nullptr) {
            return false;
        }

        // A provider can always replace its own task with a new challenger
        if (incumbent->requester_id == challenger->requester_id) {
            return true;
        }

        // Function to get the priorities of the ancestors of this task
        auto get_ancestor_priorities = [this](const std::shared_ptr<BehaviourTask>& task) {
            // We are our first ancestor
            std::vector<TaskPriority> ancestors;

            // The task might be a root task, in which case we won't have any provider
            // In that case we set the type to nullptr_t to indicate that we are a root task
            ancestors.emplace_back(
                providers.count(task->requester_id) != 0 ? providers.at(task->requester_id)->type : typeid(nullptr_t),
                task->priority,
                task->optional);

            // Loop up through the providers until we reach a point where a task was emitted by a root provider
            for (auto t = task; providers.at(t->requester_id)->classification != Provider::Classification::ROOT;) {

                // Get the provider that emitted this task, and from that the provider group
                auto provider = providers.at(t->requester_id);
                auto& group   = provider->group;

                // If there is no active task something has gone wrong with the algorithm
                if (group.active_task == nullptr) {
                    throw std::runtime_error("Task has broken parentage");
                    // TODO here is where you need to change things
                    // TODO now having no active/task no provider signifies a root element
                }

                // Add this new parent we found and set t so we can loop up further
                t = group.active_task;
                ancestors.emplace_back(provider->type, t->priority, t->optional);
            }
            return ancestors;
        };

        // Get the ancestor priorities of the two tasks
        auto i_p = get_ancestor_priorities(incumbent);
        auto c_p = get_ancestor_priorities(challenger);

        // Remove all of the common ancestors
        // If one of the lists ends up empty that means that it is a direct ancestor of the other
        // If either ends up as nullptr_t that means that it is a root task and we don't remove it
        while ((!i_p.empty() && !c_p.empty())
               && (i_p.back().requester != typeid(nullptr_t) && c_p.back().requester != typeid(nullptr_t))
               && (i_p.back().requester == c_p.back().requester)) {
            i_p.pop_back();
            c_p.pop_back();
        }

        // If the challenger parentage is empty it means that the challenger is a direct ancestor of the incumbent.
        // This can happen when we are checking tasks with a `Needs` relationship against a task higher in the tree and
        // we already own that task. In that case we say that the challenger wins because we already own it and can
        // replace it.
        // TLDR parent beats child
        if (c_p.empty()) {
            return true;
        }

        // If the incumbent is empty it means that the incumbent is a direct ancestor of the challenger.
        // As an inverse to the line above, we can't challenge our parents to ensure sane sorting order
        if (i_p.empty()) {
            return false;
        }

        // Work out if there are any optionals in either of the tasks parentage
        const bool i_o = std::any_of(i_p.begin(), i_p.end(), [](const auto& v) { return v.optional; });
        const bool c_o = std::any_of(c_p.begin(), c_p.end(), [](const auto& v) { return v.optional; });

        // If both or neither are optional then we compare at the point where their ancestors were siblings.
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
