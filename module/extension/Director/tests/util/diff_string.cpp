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

#include "diff_string.hpp"

#include <fmt/format.h>

#include "lcs.hpp"

namespace util {

    std::string diff_string(const std::vector<std::string>& expected, const std::vector<std::string>& actual) {
        // Find the longest string in each side or "Expected" and "Actual" if those are the longest
        auto len      = [](const std::string& a, const std::string& b) { return a.size() < b.size(); };
        auto max_a_it = std::max_element(expected.begin(), expected.end(), len);
        auto max_b_it = std::max_element(actual.begin(), actual.end(), len);
        int max_a     = int(std::max(std::strlen("Expected"), max_a_it != expected.end() ? max_a_it->size() : 0));
        int max_b     = int(std::max(std::strlen("Actual"), max_b_it != actual.end() ? max_b_it->size() : 0));

        // Start with a header
        std::string output = fmt::format("{:<{}}    |    {:>{}}\n", "Expected", max_a, "Actual", max_b);

        // Print a divider characters for a divider
        for (int i = 0; i < 9 + max_a + max_b; ++i) {
            output += "-";
        }
        output += "\n";

        auto [match_a, match_b] = lcs(expected, actual);
        int i_a                 = 0;
        int i_b                 = 0;
        while (i_a < int(expected.size()) && i_b < int(actual.size())) {
            if (match_a[i_a]) {
                if (match_b[i_b]) {
                    output += fmt::format("{:<{}}   <->   {:<{}}\n", expected[i_a++], max_a, actual[i_b++], max_b);
                }
                else {
                    output += fmt::format("{:<{}}   <->   {:<{}}\n", "", max_a, actual[i_b++], max_b);
                }
            }
            else {
                output += fmt::format("{:<{}}   <->   {:<{}}\n", expected[i_a++], max_a, "", max_b);
            }
        }
        while (i_a < int(expected.size())) {
            output += fmt::format("{:<{}}   <->   {:<{}}\n", expected[i_a++], max_a, "", max_b);
        }
        while (i_b < int(actual.size())) {
            output += fmt::format("{:<{}}   <->   {:<{}}\n", "", max_a, actual[i_b++], max_b);
        }

        return output;
    }

}  // namespace util
