#include "diff_string.hpp"

#include <fmt/format.h>

#include "utility/algorithm/lcs.hpp"

namespace util {

    std::string diff_string(const std::vector<std::string>& expected, const std::vector<std::string>& actual) {

        // Find the longest string in each side or "Expected" and "Actual" if those are the longest
        auto len      = [](const std::string& a, const std::string& b) { return a.size() < b.size(); };
        auto max_a_it = std::max_element(expected.begin(), expected.end(), len);
        auto max_b_it = std::max_element(actual.begin(), actual.end(), len);
        int max_a     = int(std::max(std::strlen("Expected"), max_a_it != expected.end() ? max_a_it->size() : 0));
        int max_b     = int(std::max(std::strlen("Actual"), max_b_it != expected.end() ? max_b_it->size() : 0));

        // Start with a header
        std::string output = fmt::format("{:<{}}    |    {:>{}}\n", "Expected", max_a, "Actual", max_b);

        // Print a divider characters for a divider
        output += std::string(9 + max_a + max_b, '-') + "\n";

        auto [match_a, match_b] = utility::algorithm::lcs(expected, actual);
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
