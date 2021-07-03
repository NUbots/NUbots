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

    bool Director::challenge_priority(const std::shared_ptr<const DirectorTask>& incumbent,
                                      const std::shared_ptr<const DirectorTask>& challenger) {

        // Compare to yourself and you get that you shouldn't change tasks
        if (incumbent->requester_id == challenger->requester_id) {
            return false;
        }

        // Function to get the priorities of the ancestors of this task
        auto get_ancestor_priorities = [this](const std::shared_ptr<const DirectorTask>& task) {
            // We are our first anscestor
            std::vector<std::tuple<uint64_t, int, bool>> ancestors;
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

        // Convenience references for the two to make the code shorter
        const auto& a = incumbent;
        const auto& b = challenger;

        // Get the ancestor priorities of a and b
        auto a_p = get_ancestor_priorities(a);
        auto b_p = get_ancestor_priorities(b);

        // Remove all of the common ancestors
        while (std::get<0>(a_p.back()) == std::get<0>(b_p.back())) {
            a_p.pop_back();
            b_p.pop_back();
        }

        // Work out if there are any optionals in either of the tasks parentage
        const bool a_optional = std::any_of(a_p.begin(), a_p.end(), [](const auto& v) { return std::get<2>(v); });
        const bool b_optional = std::any_of(b_p.begin(), b_p.end(), [](const auto& v) { return std::get<2>(v); });

        // If both or neither are optional then we compare at the point where they were siblings.
        // If both are optional then we would rather that whichever ancestor had higher priority have its optional tasks
        if (a_optional == b_optional) {
            return std::get<1>(a_p.back()) < std::get<1>(b_p.back());
        }

        // If the optionals are not equal then return if a_optional is true. a being optional would make it less than b
        return a_optional;
    }

}  // namespace module::extension
