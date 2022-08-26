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

#include "Director.hpp"

namespace module::extension {

    using ::extension::behaviour::commands::BehaviourTask;
    using ::module::extension::provider::Provider;
    using ::module::extension::provider::ProviderGroup;


    Director::Solution::Option Director::solve_provider(const std::shared_ptr<Provider>& provider,
                                                        const std::shared_ptr<const BehaviourTask>& authority,
                                                        std::set<std::shared_ptr<const Provider>> visited) {


        Solution::Option option;
        option.provider = provider;
        option.state    = Solution::Option::OK;

        // This prevents us going in a loop, if we have already looked at this provider in our tree in the past stop
        if (!visited.insert(provider).second) {
            option.state = Solution::Option::BLOCKED_LOOP;
            return option;
        }

        // We need to have priority over the currently running task
        auto group = groups.at(provider->type);
        if (challenge_priority(group.active_task, authority)) {
            option.state = Solution::Option::BLOCKED_PRIORITY;
            return option;
        }

        // Add each unmet when as a requirement
        for (const auto& w : provider->when) {
            if (!w->current) {
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
                                            const std::shared_ptr<const BehaviourTask>& authority,
                                            const std::set<std::shared_ptr<const Provider>>& visited) {
        Solution s;
        s.pushed = true;

        // Check all the candidates that provide a solution to this when condition
        // in the event that there are none, this solution will have no options and therefore be blocked
        for (auto& group : groups) {
            auto& g = group.second;

            // A provider already needs to be running to push it
            if (g.active_task != nullptr) {
                for (const auto& p : g.providers) {
                    if (p->causing.count(when.type) != 0 && when.validator(p->causing[when.type])) {
                        if (challenge_priority(g.pushing_task, authority)) {
                            s.options.push_back(solve_provider(p, authority, visited));
                        }
                        else {
                            // TODO Don't have the priority to push, but we could queue to push?
                        }
                    }
                }
            }
        }

        return s;
    }

    Director::Solution Director::solve_group(const std::type_index& type,
                                             const std::shared_ptr<const BehaviourTask>& authority,
                                             const std::set<std::shared_ptr<const Provider>>& visited) {
        Solution s;
        s.pushed = false;

        // Continue building the tree recursively for all the providers that meet our needs
        if (groups.count(type) != 0) {
            for (const auto& p : groups.at(type).providers) {
                s.options.push_back(solve_provider(p, authority, visited));
            }
        }

        return s;
    }

    Director::Solution Director::solve_task(const std::shared_ptr<const BehaviourTask>& task) {
        std::set<std::shared_ptr<const Provider>> visited;
        return solve_group(task->type, task, visited);
    }

}  // namespace module::extension
