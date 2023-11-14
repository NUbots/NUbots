#include "FileLogHandler.hpp"

#include "extension/Configuration.hpp"

#include "utility/strutil/ansi.hpp"
#include "utility/support/evil/pure_evil.hpp"

namespace module::support::logging {

    using NUClear::message::LogMessage;
    using NUClear::message::ReactionStatistics;

    using extension::Configuration;

    using utility::strutil::Colour;

    FileLogHandler::FileLogHandler(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , log_file_name("/home/nubots/log")
        , log_file(log_file_name, std::ios_base::out | std::ios_base::app | std::ios_base::ate) {

        on<Configuration>("FileLogHandler.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FileLogHandler.yaml
            log_file_name = config["log_file"].as<std::string>();

            if (log_file.is_open()) {
                log_file.close();
            }

            log_file.open(log_file_name, std::ios_base::out | std::ios_base::app | std::ios_base::ate);

            log_file << "\n*********************************************************************\n" << std::endl;
        });

        on<Shutdown>().then([this] {
            if (log_file.is_open()) {
                log_file.close();
            }
        });

        on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {
            if (stats.exception) {

                std::lock_guard<std::mutex> lock(mutex);

                // Get our reactor name
                std::string reactor = stats.identifiers.reactor;

                // Strip to the last semicolon if we have one
                size_t last_c = reactor.find_last_of(':');
                reactor       = last_c == std::string::npos ? reactor : reactor.substr(last_c + 1);

#ifndef NDEBUG  // We have a cold hearted monstrosity that got built!

                // Print our exception detals
                log_file << reactor << " "
                         << (stats.identifiers.name.empty() ? "" : "- " + stats.identifiers.name + " ")
                         << Colour::brightred << "Exception:"
                         << " " << Colour::brightred << utility::support::evil::exception_name << std::endl;

                // Print our stack trace
                for (auto& s : utility::support::evil::stack) {
                    log_file << "\t" << Colour::brightmagenta << s.file << ":" << Colour::brightmagenta << s.lineno
                             << " " << s.function << std::endl;
                }
#else
                try {
                    std::rethrow_exception(stats.exception);
                }
                catch (const std::exception& ex) {

                    std::string exception_name = NUClear::util::demangle(typeid(ex).name());

                    log_file << reactor << " "
                             << (stats.identifiers.name.empty() ? "" : "- " + stats.identifiers.name + " ")
                             << Colour::brightred << "Exception:"
                             << " " << Colour::brightred << exception_name << " " << ex.what() << std::endl;
                }
                // We don't actually want to crash
                catch (...) {

                    log_file << reactor << " "
                             << (stats.identifiers.name.empty() ? "" : "- " + stats.identifiers.name + " ")
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
            if (message.task != nullptr) {
                // Get our reactor name
                std::string reactor = message.task->identifiers.reactor;

                // Strip to the last semicolon if we have one
                size_t last_c = reactor.find_last_of(':');
                reactor       = last_c == std::string::npos ? reactor : reactor.substr(last_c + 1);

                // This is our source
                source = reactor + " "
                         + (message.task->identifiers.name.empty() ? "" : "- " + message.task->identifiers.name + " ");
            }

            // Output the level
            switch (message.level) {
                case NUClear::TRACE: log_file << source << "TRACE: "; break;
                case NUClear::DEBUG: log_file << source << Colour::green << "DEBUG: "; break;
                case NUClear::INFO: log_file << source << Colour::brightblue << "INFO: "; break;
                case NUClear::WARN: log_file << source << Colour::yellow << "WARN: "; break;
                case NUClear::ERROR: log_file << source << Colour::brightred << "ERROR: "; break;
                case NUClear::UNKNOWN:;
                case NUClear::FATAL: log_file << source << Colour::brightred << "FATAL: "; break;
            }

            // Output the message
            log_file << message.message << std::endl;
        });
    }
}  // namespace module::support::logging
