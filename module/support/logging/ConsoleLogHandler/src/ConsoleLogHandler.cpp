/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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

#include <iostream>
#include <regex>

#include "utility/strutil/ansi.hpp"
#include "utility/support/evil/pure_evil.hpp"

namespace module::support::logging {

    using NUClear::message::LogMessage;
    using NUClear::message::ReactionStatistics;
    using utility::strutil::Colour;

    void ConsoleLogHandler::print_log(const NUClear::LogLevel& level,
                                      const std::string& reactor,
                                      const std::string& name,
                                      const std::string& message,
                                      const std::vector<utility::support::evil::StackFrame>& stack_trace) {
        std::lock_guard<std::mutex> lock(mutex);

        // Remove the namespace from the Reactor with a regex
        auto simple_reactor = std::regex_replace(reactor, std::regex("[A-Za-z_][A-Za-z0-9_]+::"), "");
        simple_reactor      = simple_reactor.empty() ? "PowerPlant" : simple_reactor;

        // Reactor - <name>
        std::cerr << (simple_reactor + (name.empty() ? "" : " - " + name));
        std::cerr << " ";

        // Output the level
        switch (level) {
            case NUClear::TRACE: std::cerr << "TRACE: "; break;
            case NUClear::DEBUG: std::cerr << Colour::green << "DEBUG: "; break;
            case NUClear::INFO: std::cerr << Colour::brightblue << "INFO: "; break;
            case NUClear::WARN: std::cerr << Colour::yellow << "WARN: "; break;
            case NUClear::ERROR: std::cerr << Colour::brightred << "(╯°□°）╯︵ ┻━┻: "; break;
            case NUClear::UNKNOWN:  // Unknown shouldn't happen so we treat it as an error
            case NUClear::FATAL: std::cerr << Colour::brightred << "(ノಠ益ಠ)ノ彡┻━┻: "; break;
        }

        // Output the message
        std::cerr << message << std::endl;

        // Output the stack trace
        for (const auto& s : stack_trace) {
            std::cerr << "\t" << Colour::brightmagenta << s.file << ":" << Colour::brightmagenta << s.lineno << " "
                      << s.function << std::endl;
        }
    }

    std::string exception_what(const std::exception_ptr& ptr) {
        try {
            std::rethrow_exception(ptr);
        }
        catch (const std::exception& ex) {
            return ex.what();
        }
        catch (...) {
            return utility::support::evil::last_exception_name();
        }
    }

    ConsoleLogHandler::ConsoleLogHandler(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {
        on<Trigger<ReactionStatistics>, Inline::ALWAYS>().then([this](const ReactionStatistics& stats) {
            if (stats.exception) {

                // Get our reactor name and reaction name
                std::string reactor = stats.identifiers->reactor;
                std::string name    = stats.identifiers->name;

                // Get the exception string
                std::string msg = exception_what(stats.exception);

                // Make sure we are on the thread that the exception was thrown on otherwise this will not work
                auto stack_trace = utility::support::evil::last_exception_stack_trace();

                print_log(NUClear::FATAL, reactor, name, msg, stack_trace);
            }
        });

        on<Trigger<LogMessage>>().then([this](const LogMessage& message) {
            if (message.level < message.display_level) {
                return;
            }

            // Grab identifiers if they exist
            const auto& ids = message.statistics != nullptr ? message.statistics->identifiers : nullptr;

            // Where this message came from
            std::string reactor = ids != nullptr ? ids->reactor : "NUClear::PowerPlant";
            std::string name    = ids != nullptr ? ids->name : "";

            print_log(message.level, reactor, name, message.message, {});
        });
    }

}  // namespace module::support::logging
