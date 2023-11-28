/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ConsoleLogHandler.hpp"

#include "utility/strutil/ansi.hpp"
#include "utility/support/evil/pure_evil.hpp"

namespace module::support::logging {

    using NUClear::message::LogMessage;
    using NUClear::message::ReactionStatistics;
    using utility::strutil::Colour;

    ConsoleLogHandler::ConsoleLogHandler(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {
        on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {
            if (stats.exception) {

                std::lock_guard<std::mutex> lock(mutex);

                // Get our reactor name
                std::string reactor = stats.identifiers.reactor;

                // Strip to the last semicolon if we have one
                size_t lastC = reactor.find_last_of(':');
                reactor      = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

#ifndef NDEBUG  // We have a cold hearted monstrosity that got built!

                std::string exception_what;
                try {
                    std::rethrow_exception(stats.exception);
                }
                catch (const std::exception& ex) {
                    exception_what = ex.what();
                }
                // We don't actually want to crash
                catch (...) {
                }

                // Print our exception details
                std::cerr << reactor << " "
                          << (stats.identifiers.name.empty() ? "" : "- " + stats.identifiers.name + " ")
                          << Colour::brightred << "(╯°□°）╯︵ ┻━┻ "
                          << " " << Colour::brightred << utility::support::evil::exception_name << " " << exception_what
                          << std::endl;

                // Print our stack trace
                for (auto& s : utility::support::evil::stack) {
                    std::cerr << "\t" << Colour::brightmagenta << s.file << ":" << Colour::brightmagenta << s.lineno
                              << " " << s.function << std::endl;
                }
#else
                try {
                    std::rethrow_exception(stats.exception);
                }
                catch (const std::exception& ex) {

                    std::string exceptionName = NUClear::util::demangle(typeid(ex).name());

                    std::cerr << reactor << " "
                              << (stats.identifiers.name.empty() ? "" : "- " + stats.identifiers.name + " ")
                              << Colour::brightred << "(╯°□°）╯︵ ┻━┻ "
                              << " " << Colour::brightred << exceptionName << " " << ex.what() << std::endl;
                }
                // We don't actually want to crash
                catch (...) {

                    std::cerr << reactor << " "
                              << (stats.identifiers.name.empty() ? "" : "- " + stats.identifiers.name + " ")
                              << Colour::brightred << "(ノಠ益ಠ)ノ彡┻━┻" << std::endl;
                }
#endif
            }
        });


        on<Trigger<LogMessage>>().then([this](const LogMessage& message) {
            // Only display messages that are above the display level of the reactor that made the log
            if (message.level < message.display_level) {
                return;
            };
            std::lock_guard<std::mutex> lock(mutex);

            // Where this message came from
            std::string source = "";

            // If we know where this log message came from, we display that
            if (message.task != nullptr) {
                // Get our reactor name
                std::string reactor = message.task->identifiers.reactor;

                // Strip to the last semicolon if we have one
                size_t lastC = reactor.find_last_of(':');
                reactor      = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

                // This is our source
                source = reactor + " "
                         + (message.task->identifiers.name.empty() ? "" : "- " + message.task->identifiers.name + " ");
            }

            // Output the level
            switch (message.level) {
                case NUClear::TRACE: std::cerr << source << "TRACE: "; break;
                case NUClear::DEBUG: std::cerr << source << Colour::green << "DEBUG: "; break;
                case NUClear::INFO: std::cerr << source << Colour::brightblue << "INFO: "; break;
                case NUClear::WARN: std::cerr << source << Colour::yellow << "WARN: "; break;
                case NUClear::ERROR: std::cerr << source << Colour::brightred << "(╯°□°）╯︵ ┻━┻: "; break;
                case NUClear::UNKNOWN:;
                case NUClear::FATAL: std::cerr << source << Colour::brightred << "(ノಠ益ಠ)ノ彡┻━┻: "; break;
            }

            // Output the message
            std::cerr << message.message << std::endl;
        });
    }

}  // namespace module::support::logging
