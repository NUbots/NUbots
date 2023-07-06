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

    using component::Provider;

    Director::PushedSolution Director::filter_deepest_and_merge(const std::vector<Director::PushedSolution>& sols) {

        // Only the solutions at the deepest level matter
        int deepest = std::max_element(sols.begin(), sols.end(), [](const auto& a, const auto& b) {
                          a.providers.empty() ? true : b.providers.empty() ? false : a.level < b.level;
                      })->level;

        // Make a per solution per group list of providers for the deepest level
        std::vector<std::map<std::type_index, std::set<std::shared_ptr<Provider>>>> split_groups;
        for (const auto& s : sols) {
            split_groups.emplace_back();
            auto& groups = split_groups.back();
            if (s.level == deepest) {
                for (const auto& p : s.providers) {
                    groups[p->type].insert(p);
                }
            }
        }

        // Merge the providers for each group by taking the intersection of all the providers as our final solution
        // Taking the intersection ensures that any of the group providers left in the intersection will work for all of
        // providers requirements
        std::map<std::type_index, std::set<std::shared_ptr<Provider>>> merged_groups;
        for (const auto& groups : split_groups) {
            for (const auto& [type, providers] : groups) {
                if (merged_groups.count(type) == 0) {
                    merged_groups[type] = providers;
                }
                else {
                    std::set<std::shared_ptr<Provider>> intersection;
                    std::set_intersection(merged_groups[type].begin(),
                                          merged_groups[type].end(),
                                          providers.begin(),
                                          providers.end(),
                                          std::inserter(intersection, intersection.begin()));
                    if (intersection.empty()) {
                        return PushedSolution{true, deepest, {}};
                    }
                    merged_groups[type] = intersection;
                }
            }
        }

        // Merge the groups into a single set of providers
        PushedSolution result{false, deepest, {}};
        for (const auto& [type, providers] : merged_groups) {
            result.providers.insert(providers.begin(), providers.end());
        }
        return result;
    }

    Director::PushedSolution Director::find_pushing_solution(const Solution& requirement, const int& pushing_depth) {

        // Analyse the options so we can accumulate the best pushes to do
        std::vector<PushedSolution> push;
        int shallowest = std::numeric_limits<int>::max();
        for (const auto& option : requirement.options) {

            // See if this option needs to be pushed to work
            auto s = find_pushing_solution(option, pushing_depth + requirement.pushed ? 1 : 0);

            // Only add non blocked options to the list
            if (!s.blocked) {
                // This option wants to push something
                if (!s.providers.empty()) {
                    push.push_back(s);
                    shallowest = std::min(shallowest, s.level);
                }
                // If we are a pushed solution, then each of our options are pushed options
                else if (requirement.pushed) {
                    push.push_back(PushedSolution{false, pushing_depth, {option.provider}});
                    shallowest = std::min(shallowest, pushing_depth);
                }
                else {
                    // We found an option that doesn't need to be pushed so return that we can run without pushing
                    return PushedSolution{false, pushing_depth, {}};
                }
            }
        }

        // No options means we are blocked
        if (push.empty()) {
            return PushedSolution{true, pushing_depth, {}};
        }

        PushedSolution result{false, shallowest, {}};
        for (const auto& p : push) {
            if (p.level == shallowest) {
                result.providers.insert(p.providers.begin(), p.providers.end());
            }
        }
        return result;
    }

    Director::PushedSolution Director::find_pushing_solution(const Solution::Option& option, const int& pushing_depth) {

        // We can handle when things are OK and when things are blocked by when
        if (option.state == Solution::Option::OK || option.state == Solution::Option::BLOCKED_WHEN) {
            // Join all the requirements together and then reduce to only the highest level
            // For this option, the highest level must be met for this option to be viable
            std::vector<PushedSolution> requirements;
            for (const auto& r : option.requirements) {
                auto s = find_pushing_solution(r, pushing_depth);

                // If any requirement is blocked, everything is blocked
                if (s.blocked) {
                    return PushedSolution{true, pushing_depth, {}};
                }

                // If there were any pushes required, add them to the list
                if (!s.providers.empty()) {
                    requirements.push_back(s);
                }
            }

            // No requirements means we don't need to push
            if (requirements.empty()) {
                return PushedSolution{false, pushing_depth, {}};
            }

            return filter_deepest_and_merge(requirements);
        }

        // We are blocked for some other reason so we can't be a pushing solution
        return PushedSolution{true, pushing_depth, {}};
    }

    Director::PushedSolution Director::find_pushing_solutions(const std::vector<Solution>& solutions) {

        // Get pushing solutions for each solution
        std::vector<PushedSolution> pushed_solutions;
        for (const auto& solution : solutions) {
            // Choose an option
            auto s = find_pushing_solution(solution, 0);

            // If any of the solutions is blocked, then return no solutions
            if (s.blocked) {
                return PushedSolution{true, 0, {}};
            }

            pushed_solutions.push_back(s);
        }

        return filter_deepest_and_merge(pushed_solutions);
    }

}  // namespace module::extension
