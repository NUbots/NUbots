#ifndef MODULE_SUPPORT_LEGLOADSLOGGER_H
#define MODULE_SUPPORT_LEGLOADSLOGGER_H

#include <fstream>
#include <nuclear>
#include <string>

namespace module {
namespace support {
    namespace logging {

        class LegLoadsLogger : public NUClear::Reactor {

        public:
            /// @brief Called by the powerplant to build and setup the LegLoadsLogger reactor.
            explicit LegLoadsLogger(std::unique_ptr<NUClear::Environment> environment);

        private:
            std::ofstream logFile;
            std::string logFilePath;
        };
    }
}
}

#endif  // MODULE_SUPPORT_LEGLOADSLOGGER_H
