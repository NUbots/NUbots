#include "FileLogHandler.h"

#include "extension/Configuration.h"
#include "utility/strutil/ansi.h"
#include "utility/support/evil/pure_evil.h"

namespace module {
namespace support {
    namespace logging {

        using NUClear::message::LogMessage;
        using NUClear::message::ReactionStatistics;

        using extension::Configuration;

        using utility::strutil::Colour;

        FileLogHandler::FileLogHandler(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , mutex()
            , logFileName("/home/nubots/log")
            , logFile(logFileName, std::ios_base::out | std::ios_base::app | std::ios_base::ate) {

            on<Configuration>("FileLogHandler.yaml").then([this](const Configuration& config) {
                // Use configuration here from file FileLogHandler.yaml
                logFileName = config["log_file"].as<std::string>();

                if (logFile.is_open()) {
                    logFile.close();
                }

                logFile.open(logFileName, std::ios_base::out | std::ios_base::app | std::ios_base::ate);
            });

            on<Shutdown>().then([this] {
                if (logFile.is_open()) {
                    logFile.close();
                }
            });

            on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {
                if (stats.exception) {

                    std::lock_guard<std::mutex> lock(mutex);

                    // Get our reactor name
                    std::string reactor = stats.identifier[1];

                    // Strip to the last semicolon if we have one
                    size_t lastC = reactor.find_last_of(':');
                    reactor      = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

#ifndef NDEBUG  // We have a cold hearted monstrosity that got built!

                    // Print our exception detals
                    logFile << reactor << " " << (stats.identifier[0].empty() ? "" : "- " + stats.identifier[0] + " ")
                            << Colour::brightred << "Exception:"
                            << " " << Colour::brightred << utility::support::evil::exception_name << std::endl;

                    // Print our stack trace
                    for (auto& s : utility::support::evil::stack) {
                        logFile << "\t" << Colour::brightmagenta << s.file << ":" << Colour::brightmagenta << s.lineno
                                << " " << s.function << std::endl;
                    }
#else
                    try {
                        std::rethrow_exception(stats.exception);
                    }
                    catch (const std::exception& ex) {

                        std::string exceptionName = NUClear::util::demangle(typeid(ex).name());

                        logFile << reactor << " "
                                << (stats.identifier[0].empty() ? "" : "- " + stats.identifier[0] + " ")
                                << Colour::brightred << "Exception:"
                                << " " << Colour::brightred << exceptionName << " " << ex.what() << std::endl;
                    }
                    // We don't actually want to crash
                    catch (...) {

                        logFile << reactor << " "
                                << (stats.identifier[0].empty() ? "" : "- " + stats.identifier[0] + " ")
                                << Colour::brightred << "Exception of unkown type" << std::endl;
                    }
#endif
                }
            });

            on<Trigger<LogMessage>>().then([this](const LogMessage& message) {
                std::lock_guard<std::mutex> lock(mutex);

                // Where this message came from
                std::string source = "";

                // If we know where this log message came from, we display that
                if (message.task) {
                    // Get our reactor name
                    std::string reactor = message.task->identifier[1];

                    // Strip to the last semicolon if we have one
                    size_t lastC = reactor.find_last_of(':');
                    reactor      = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

                    // This is our source
                    source = reactor + " "
                             + (message.task->identifier[0].empty() ? "" : "- " + message.task->identifier[0] + " ");
                }

                // Output the level
                switch (message.level) {
                    case NUClear::TRACE: logFile << source << "TRACE: "; break;
                    case NUClear::DEBUG: logFile << source << Colour::green << "DEBUG: "; break;
                    case NUClear::INFO: logFile << source << Colour::brightblue << "INFO: "; break;
                    case NUClear::WARN: logFile << source << Colour::yellow << "WARN: "; break;
                    case NUClear::ERROR: logFile << source << Colour::brightred << "ERROR: "; break;
                    case NUClear::FATAL: logFile << source << Colour::brightred << "FATAL: "; break;
                }

                // Output the message
                logFile << message.message << std::endl;
            });
        }
    }  // namespace logging
}  // namespace support
}  // namespace module
