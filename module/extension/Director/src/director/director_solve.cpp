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

    using ::extension::behaviour::commands::DirectorTask;
    using ::module::extension::provider::Provider;
    using ::module::extension::provider::ProviderGroup;

    std::shared_ptr<Provider> Director::find_push(const std::shared_ptr<const DirectorTask>& authority,
                                                  const Provider::WhenCondition& when) {

        // Search all the provider groups
        for (const auto& gp : groups) {
            auto& g = gp.second;

            // We can only push if there is an active task and we have higher priority than what's running in the task
            // Also if this task is already being pushed we have to have higher priority than the pusher
            if (g.active_task != nullptr && challenge_priority(g.active_task, authority)
                && (g.pushing_task == nullptr || challenge_priority(g.pushing_task, authority))) {

                // See if we can find a causing that can help us
                for (const auto& p : g.providers) {
                    if (p->causing.count(when.type) != 0 && when.validator(p->causing[when.type])) {

                        // TODO this is where stuff can get crazy...
                        // We need to work out if we can run this provider or not (it's own when conditions etc)
                        // We already know that this provider group can run since it has an active task but this
                        // specific provider might not be able to run. When trying to solve this though we might find
                        // that we can push yet another module to allow this to run.

                        // We have to be careful about things like loops here, so we need to make sure we don't
                        // never find a solution and get stuck in a loop

                        return p;
                    }
                }
            }
        }

        // We couldn't find a provider that we can push that will do what we want
        return nullptr;
    }

    Director::Solution Director::director_solution(const std::shared_ptr<const DirectorTask>& task) {
        return director_solution(task, task->type);
    }

    Director::Solution Director::director_solution(const std::shared_ptr<const DirectorTask>& authority,
                                                   const std::type_index& requester) {

        // Get the group
        auto& group = groups[requester];

        // We need to have priority over the currently running task
        if (!challenge_priority(group.active_task, authority)) {
            return {Solution::Type::BLOCKED, {}};
        }

        // We will build up a list of solutions so we can work out which one is best later
        std::vector<Solution> solutions;

        // Find a provider option that we are able to execute
        for (auto& provider : group.providers) {
            Solution solution;
            solution.type = Solution::Type::SOLUTION;

            // Check all the when conditions to determine if we can execute this provider
            for (const auto& w : provider->when) {

                // We continue to have success, build up the solution
                if (w->current && solution.type == Solution::Type::SOLUTION) {
                    solution.providers.push_back(provider);
                }
                // We can't meet this when condition, find out if we can push another group
                else if (!w->current) {
                    // See if we can find a pushable provider that can help us out
                    std::shared_ptr<Provider> provider = find_push(authority, *w);

                    // We have pushable provider
                    if (provider != nullptr) {
                        // We no longer have a solution
                        if (solution.type == Solution::Type::SOLUTION) {
                            solution.providers.clear();
                            solution.type = Solution::Type::PUSHABLE;
                        }
                        solution.providers.push_back(provider);
                    }
                    // We are blocked, clear list, set to blocked and break
                    else {
                        solution.type = Solution::Type::BLOCKED;
                        solution.providers.clear();
                        break;
                    }
                }
            }

            // If we are already blocked, don't bother searching our `needs` requirements
            if (solution.type != Solution::Type::BLOCKED) {

                // Check each of the `needs` types recursively
                for (const auto& n : provider->needs) {
                    auto sub = director_solution(authority, n);

                    // If blocked this whole thing is blocked, break the loop
                    if (sub.type == Solution::Type::BLOCKED || solution.type == Solution::Type::BLOCKED) {
                        solution.type = Solution::Type::BLOCKED;
                        solution.providers.clear();
                        break;
                    }

                    // We are continuing on the same path both are solution or both are pushable
                    else if ((solution.type == Solution::Type::SOLUTION && sub.type == Solution::Type::SOLUTION)
                             || (solution.type == Solution::Type::PUSHABLE && sub.type == Solution::Type::PUSHABLE)) {
                        solution.providers.insert(solution.providers.end(), sub.providers.begin(), sub.providers.end());
                    }

                    // We had a solution but this new `needs` requires pushing
                    else if (solution.type == Solution::Type::SOLUTION && sub.type == Solution::Type::PUSHABLE) {
                        solution.type = Solution::Type::PUSHABLE;
                        // Swap to the pushed providers rather than the solution providers
                        solution.providers = sub.providers;
                    }

                    // If we are pushable but our `needs` is fine ignore it and continue with our pushed provider output
                    else if (solution.type == Solution::Type::PUSHABLE && sub.type == Solution::Type::SOLUTION) {
                        // Ignore
                    }
                }
                if (solution.type != Solution::Type::BLOCKED) {
                    solutions.push_back(std::move(solution));
                }
            }
        }

        // No solution :(
        if (solutions.empty()) {
            return {Solution::Type::BLOCKED, {}};
        }

        // Solution is best, then Pushable, shouldn't see blocked here but I suppose that's worst
        return *std::min_element(solutions.begin(), solutions.end(), [](const Solution& a, const Solution& b) {
            return a.type < b.type;
        });
    }
}  // namespace module::extension
