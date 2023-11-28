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
