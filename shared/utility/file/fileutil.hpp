#ifndef UTILITY_FILEUTIL_HPP
#define UTILITY_FILEUTIL_HPP

#include <array>
#include <chrono>
#include <fstream>
#include <string>
#include <vector>

namespace utility::file {

    /**
     * @brief Splits a path into it's basename and dirname components.
     *
     * @param input the input string
     *
     * @return the dirname and basename in the posix style
     */
    std::pair<std::string, std::string> pathSplit(const std::string& input);

}  // namespace utility::file
#endif
