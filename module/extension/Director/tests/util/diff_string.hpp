
#include <string>
#include <vector>

namespace util {

    /**
     * @brief Using an LCS algorithm prints out the two sets of string (expected and actual) side by side to show the
     * differences
     *
     * @param expected  the expected series of events
     * @param actual    the actual series of events
     *
     * @return a multiline string showing a human output of the difference
     */
    std::string diff_string(const std::vector<std::string>& expected, const std::vector<std::string>& actual);

}  // namespace util
