/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "ConsoleLogHandler.h"

#include "utility/strutil/ansi.h"

namespace module {
    namespace support {
        namespace logging {

            using NUClear::message::LogMessage;
            using NUClear::message::ReactionStatistics;
            using utility::strutil::Colour;

            ConsoleLogHandler::ConsoleLogHandler(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics & stats) {
                    if (stats.exception) {
                        try {
                            std::rethrow_exception(stats.exception);
                        }
                        catch (const std::exception& ex) {


                            for (auto stat : stats.identifier) {
                                NUClear::log<NUClear::ERROR>("Identifier:", stat);
                            }
                            NUClear::log<NUClear::ERROR>("Unhandled Exception:"
                                , NUClear::util::demangle(typeid(ex).name()),
                                ex.what());
                        }
                        // We don't actually want to crash
                        catch (...) {
                            NUClear::log<NUClear::ERROR>("Unhandled Exception of unknown type");
                        }
                    }
                });


                on<Trigger<LogMessage>>().then([this] (const LogMessage& message) {

                    std::lock_guard<std::mutex> lock(mutex);

                    // Where this message came from
                    std::string source = "";

                    // If we know where this log message came from, we display that
                    if (message.task) {
                        // Get our reactor name
                        std::string reactor = message.task->identifier[1];

                        // Strip to the last semicolon if we have one
                        size_t lastC = reactor.find_last_of(':');
                        reactor = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

                        // This is our source
                        source = reactor + " ";
                    }

                    // Output the level
                    switch(message.level) {
                        case NUClear::TRACE:
                            std::cout << source << "TRACE: ";
                            break;
                        case NUClear::DEBUG:
                            std::cout << source << Colour::green << "DEBUG: ";
                            break;
                        case NUClear::INFO:
                            std::cout << source << Colour::brightblue << "INFO: ";
                            break;
                        case NUClear::WARN:
                            std::cout << source << Colour::yellow << "WARN: ";
                            break;
                        case NUClear::ERROR:
                            std::cout << source << Colour::red << "ERROR: ";
                            break;
                        case NUClear::FATAL:
                            std::cout << source << Colour::red << "FATAL: ";
                            break;
                    }

                    // Output the message
                    std::cout << message.message << std::endl;
                });
            }

        }  // logging
    }  // support
}  // modules
