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

    Director::PushedSolution Director::filter_deepest(const std::vector<Director::PushedSolution>& sols) {

        // We push at the deepest level of our requirements
        int deepest = std::min_element(sols.begin(), sols.end(), [](const auto& a, const auto& b) {
                          return a.level < b.level;
                      })->level;

        PushedSolution result{false, deepest, {}};
        for (const auto& s : sols) {
            if (s.level == deepest) {
                result.providers.insert(s.providers.begin(), s.providers.end());
            }
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

            return filter_deepest(requirements);
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

            // If any option is blocked, everything is blocked
            if (s.blocked) {
                return PushedSolution{true, 0, {}};
            }

            pushed_solutions.push_back(s);
        }

        return filter_deepest(pushed_solutions);
    }

}  // namespace module::extension
