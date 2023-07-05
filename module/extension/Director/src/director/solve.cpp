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

    Director::Solution::Option Director::solve_provider(const std::shared_ptr<Provider>& provider,
                                                        const std::shared_ptr<DirectorTask>& authority,
                                                        std::set<std::type_index> visited) {

        Solution::Option option;
        option.provider = provider;
        option.state    = Solution::Option::OK;

        // This prevents us going in a loop, if we have already looked at this provider in our tree in the past stop
        if (!visited.insert(provider->type).second) {
            option.state = Solution::Option::BLOCKED_LOOP;
            return option;
        }

        // We need to have priority over the currently running task
        auto group = provider->group;
        if (!challenge_priority(group.active_task, authority)) {
            option.state = Solution::Option::BLOCKED_PRIORITY;
            return option;
        }

        // Add each unmet when as a requirement
        for (const auto& w : provider->when) {
            if (!w->current) {
                option.state = Solution::Option::BLOCKED_WHEN;
                option.requirements.push_back(solve_when(*w, authority, visited));
            }
        }

        // Add each needs requirement
        for (const auto& n : provider->needs) {
            option.requirements.push_back(solve_group(n, authority, visited));
        }

        return option;
    }


    Director::Solution Director::solve_when(const Provider::WhenCondition& when,
                                            const std::shared_ptr<DirectorTask>& authority,
                                            const std::set<std::type_index>& visited) {
        Solution s;
        s.pushed = true;

        // Check all the candidates that provide a solution to this when condition
        // in the event that there are none, this solution will have no options and therefore be blocked
        for (auto& group_item : groups) {
            auto& group = group_item.second;

            // A provider already needs to be running to push it
            if (group.active_task != nullptr) {

                // Add an option for everyone who could provide this when condition
                for (const auto& p : group.providers) {
                    if (p->classification == Provider::Classification::PROVIDE && p->causing.contains(when.type)
                        && when.validator(p->causing[when.type])) {
                        // We now swap to using the running providers authority
                        s.options.push_back(solve_provider(p, group.active_task, visited));

                        // If we can't beat the pushing tasks priority we are also blocked
                        if (!challenge_priority(group.pushing_task, authority)) {
                            s.options.back().state = Solution::Option::BLOCKED_PRIORITY;
                        }
                    }
                }
            }
        }

        return s;
    }

    Director::Solution Director::solve_group(const std::type_index& type,
                                             const std::shared_ptr<DirectorTask>& authority,
                                             const std::set<std::type_index>& visited) {
        Solution s;
        s.pushed = false;

        // Continue building the tree recursively for all the providers that meet our needs
        if (groups.contains(type)) {
            auto& group = groups.at(type);

            // If the group is being pushed and we can't beat the priority it limits us to the provider it pushed for
            if (group.pushing_task != nullptr && !challenge_priority(group.pushing_task, authority)) {
                s.options.push_back(solve_provider(group.pushed_provider, authority, visited));
            }
            // Otherwise we can use any provider that meets our needs
            else {
                for (const auto& p : group.providers) {
                    if (p->classification == Provider::Classification::PROVIDE && p->reaction->enabled) {
                        s.options.push_back(solve_provider(p, authority, visited));
                    }
                }
            }
        }

        return s;
    }

    Director::Solution Director::solve_task(const std::shared_ptr<DirectorTask>& task) {
        std::set<std::type_index> visited;
        return solve_group(task->type, task, visited);
    }

}  // namespace module::extension
