#ifndef UTILITY_SUPPORT_RUN_CMD_HPP
#define UTILITY_SUPPORT_RUN_CMD_HPP

#include <cstdlib>

namespace utility::support {

    /// @brief std::system without glibc's warn_unused_result, for best-effort shell commands
    inline int run_cmd(const char* cmd) {
        return std::system(cmd);
    }

}  // namespace utility::support

#endif  // UTILITY_SUPPORT_RUN_CMD_HPP
