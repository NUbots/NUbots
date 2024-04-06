/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Director.hpp"

namespace module::extension {

    using component::Provider;

    Director::OkSolution Director::find_ok_solution(const Solution& solution,
                                                    const std::set<std::type_index>& used_types) {

        // We need to accumulate blocked types for options we didn't use
        // Since we search the options in order
        std::set<std::type_index> blocking_groups;

        // Look at all the options and build up the SelectedOptions we have
        for (const auto& option : solution.options) {

            auto o = find_ok_solution(option, used_types);

            // Add any blocked types we found while evaluating this solution
            blocking_groups.insert(o.blocking_groups.begin(), o.blocking_groups.end());

            // Return the first solution we find that is not blocked
            if (!o.blocked) {
                OkSolution(false, o.provider, std::move(o.requirements), std::move(o.used), std::move(blocking_groups));
                return o;
            }
        }

        // We couldn't find an OK solution, return blocked
        return OkSolution(true, std::move(blocking_groups));
    }

    Director::OkSolution Director::find_ok_solution(const Solution::Option& option,
                                                    std::set<std::type_index> used_types) {

        // If the passed option is not an OK option it is blocked
        if (option.state != Solution::Option::OK) {
            return OkSolution(true, {option.provider->type});
        }

        // If this type was previously used by another branch we can't reuse it
        // This can happen when you try to run two sibling tasks that have mutual needs
        if (option.provider != nullptr && !used_types.insert(option.provider->type).second) {
            return OkSolution(true, {option.provider->type});
        }

        // Store the providers that are needed for each of our requirements
        std::vector<std::shared_ptr<Provider>> requirement_providers;

        // If we have requirements we need to combine their output as our solution
        bool blocked = false;
        std::set<std::type_index> blocking_groups;
        for (const auto& r : option.requirements) {

            // If this is a pushed requirement we need to watch it
            if (r.pushed) {
                blocked = true;
                blocking_groups.insert(option.provider->type);
            }
            else {
                // Choose the best option for this requirement
                auto s = find_ok_solution(r, used_types);

                blocked = blocked || s.blocked;

                // Add the blocked types to our blocked set
                blocking_groups.insert(s.blocking_groups.begin(), s.blocking_groups.end());

                // Condense the providers in this requirement into a single list
                requirement_providers.push_back({s.provider});
                requirement_providers.insert(requirement_providers.end(), s.requirements.begin(), s.requirements.end());

                // Add the used types to our list
                used_types.insert(s.used.begin(), s.used.end());
            }
        }

        return OkSolution(blocked,
                          option.provider,
                          std::move(requirement_providers),
                          std::move(used_types),
                          std::move(blocking_groups));
    }

    std::vector<Director::OkSolution> Director::find_ok_solutions(const std::vector<Solution>& solutions,
                                                                  const std::set<std::type_index>& used) {

        // Find each one individually but pass through the used types
        std::vector<OkSolution> ok_solutions;
        std::set<std::type_index> used_types = used;
        for (const auto& solution : solutions) {

            // Choose an option
            ok_solutions.push_back(find_ok_solution(solution, used_types));

            // Add the used types to our list
            used_types.insert(ok_solutions.back().used.begin(), ok_solutions.back().used.end());
        }

        return ok_solutions;
    }

}  // namespace module::extension
