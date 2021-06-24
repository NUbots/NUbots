#ifndef MODULE_SUPPORT_LOGGING_FILELOGHANDLER_HPP
#define MODULE_SUPPORT_LOGGING_FILELOGHANDLER_HPP

#include <fstream>
#include <filesystem>
#include <nuclear>
#include <string>

namespace module::support::logging {

    class FileLogHandler : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the FileLogHandler reactor.
        explicit FileLogHandler(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::filesystem::path logFileName;
        std::ofstream logFile;
        int max_size;

        /// Holds the reaction so we can disable it when we fill the log folder
        ReactionHandle logging_reaction;
    };
}  // namespace module::support::logging

#endif  // MODULE_SUPPORT_LOGGING_FILELOGHANDLER_HPP
