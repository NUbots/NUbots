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

#include <algorithm>
#include <tuple>
#include <vector>

#include "Director.hpp"

#include "extension/Behaviour.hpp"

namespace module::extension {

    using component::DirectorTask;
    using component::Provider;

    // Scope this struct to just this translation unit
    namespace {
        struct TaskPriority {
            TaskPriority(const std::type_index& requester_,
                         const int& priority_,
                         const bool& optional_,
                         const bool& dying_)
                : requester(requester_), priority(priority_), optional(optional_), dying(dying_) {}

            std::type_index requester;
            int priority;
            bool optional;
            bool dying;
        };
    }  // namespace

    bool Director::challenge_priority(const std::shared_ptr<DirectorTask>& incumbent,
                                      const std::shared_ptr<DirectorTask>& challenger) {

        // If there is no incumbent the challenger wins by default
        if (incumbent == nullptr) {
            return true;
        }
        if (challenger == nullptr) {
            return false;
        }

        // A provider of the same group can always replace its own task with a new challenger
        if (providers.at(incumbent->requester_id)->type == providers.at(challenger->requester_id)->type) {
            return true;
        }

        // Function to get the priorities of the ancestors of this task
        auto get_ancestor_priorities = [this](const std::shared_ptr<DirectorTask>& task) {
            std::vector<TaskPriority> ancestors;

            // Loop up through the providers until we reach a point where a task was emitted by a root provider
            // We recognise that we have passed a root provider when the active task is nullptr
            auto t = task;
            do {
                ancestors.emplace_back(t->type, t->priority, t->optional, t->dying);
                auto p              = providers.at(t->requester_id);
                auto classification = p->classification;
                t                   = p->group.active_task;

                // Check if we reached a nullptr task but we are not a root provider or a zombie
                if (t == nullptr) {
                    if (p->group.zombie) {
                        // Add an ancestor that is the lowest possible priority and optional
                        // Anyone can beat up a zombie, I mean, it's just a bunch of rotting flesh
                        ancestors.emplace_back(p->group.type, std::numeric_limits<int>::min(), true, true);
                    }
                    else if (classification != Provider::Classification::ROOT) {
                        throw std::runtime_error("Task has broken parentage");
                    }
                }
            } while (t != nullptr);

            return ancestors;
        };

        // Get the ancestor priorities of the two tasks
        auto i_p = get_ancestor_priorities(incumbent);
        auto c_p = get_ancestor_priorities(challenger);

        // Remove all of the common ancestors
        // If one of the lists ends up empty that means that it is a direct ancestor of the other
        // If either ends up as nullptr_t that means that it is a root task and we don't remove it
        while ((!i_p.empty() && !c_p.empty()) && (i_p.back().requester == c_p.back().requester)) {
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

        // If either, but not both, of the tasks are dying, the one that is not dying wins
        if (i_p.back().dying != c_p.back().dying) {
            return i_p.back().dying;
        }

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
