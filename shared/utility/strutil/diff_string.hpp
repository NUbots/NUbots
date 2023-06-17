#ifndef UTILITIY_STRUTIL_DIFF_STRING_HPP
#define UTILITIY_STRUTIL_DIFF_STRING_HPP

#include <string>
#include <vector>

namespace utility::strutil {

    /**
     * Using an LCS algorithm prints out the two sets of string (expected and actual) side by side to show the
     * differences
     *
     * @param expected  the expected series of events
     * @param actual    the actual series of events
     *
     * @return a multiline string showing a human output of the difference
     */
    [[nodiscard]] std::string diff_string(const std::vector<std::string>& expected,
                                          const std::vector<std::string>& actual);

}  // namespace utility::strutil

#endif  // UTILITIY_STRUTIL_DIFF_STRING_HPP
