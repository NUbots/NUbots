#ifndef MODULE_SUPPORT_LOGGING_FILELOGHANDLER_HPP
#define MODULE_SUPPORT_LOGGING_FILELOGHANDLER_HPP

#include <fstream>
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

        std::string log_file_name;
        std::ofstream log_file;
    };
}  // namespace module::support::logging

#endif  // MODULE_SUPPORT_LOGGING_FILELOGHANDLER_HPP
