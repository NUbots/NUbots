#ifndef MODULE_SUPPORT_LOGGING_FILELOGHANDLER_HPP
#define MODULE_SUPPORT_LOGGING_FILELOGHANDLER_HPP

#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <nuclear>
#include <string>

namespace module::support::logging {

    class FileLogHandler : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the FileLogHandler reactor.
        explicit FileLogHandler(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::mutex mutex;
        std::filesystem::path log_file_name = std::filesystem::path("log") / std::filesystem::path("log");
        std::ofstream log_file;
        uint64_t max_size = std::numeric_limits<uint64_t>::max();

        /// Holds the reaction so we can disable it when we fill the log folder
        ReactionHandle logging_reaction{};
        ReactionHandle stats_reaction{};

        /// Holds the reaction so we can disable it so we don't recheck the file size
        ReactionHandle log_check_handler{};
    };
}  // namespace module::support::logging

#endif  // MODULE_SUPPORT_LOGGING_FILELOGHANDLER_HPP
