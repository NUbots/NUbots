#ifndef MODULE_SUPPORT_LOGGING_FILELOGHANDLER_H
#define MODULE_SUPPORT_LOGGING_FILELOGHANDLER_H

#include <fstream>
#include <mutex>
#include <nuclear>
#include <string>

namespace module {
namespace support {
    namespace logging {

        class FileLogHandler : public NUClear::Reactor {

        public:
            /// @brief Called by the powerplant to build and setup the FileLogHandler reactor.
            explicit FileLogHandler(std::unique_ptr<NUClear::Environment> environment);

        private:
            std::mutex mutex;

            std::string logFileName;
            std::ofstream logFile;
        };
    }  // namespace logging
}  // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_LOGGING_FILELOGHANDLER_H
