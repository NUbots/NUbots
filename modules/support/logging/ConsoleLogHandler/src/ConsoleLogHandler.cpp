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

namespace modules {
    namespace support {
        namespace logging {

            ConsoleLogHandler::ConsoleLogHandler(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                on<Trigger<NUClear::ReactionStatistics>>([this](const NUClear::ReactionStatistics & stats) {
                    if (stats.exception) {
                        try {
                            std::rethrow_exception(stats.exception);
                        }
                        catch (std::exception ex) {
                            for (auto stat : stats.identifier) {
                                NUClear::log<NUClear::ERROR>("Identifier: ", stat);
                            }
                            NUClear::log<NUClear::ERROR>("Unhandled Exception: ", ex.what());
                        }
                        // We don't actually want to crash
                        catch (...) {
                        }
                    }
                });


                on<Trigger<NUClear::LogMessage>, Options<Sync<ConsoleLogHandler>>>([this](const NUClear::LogMessage& message) {

                    // Output the level
                    switch(message.level) {
                        case NUClear::TRACE:
                            std::cout << "TRACE: ";
                            break;
                        case NUClear::DEBUG:
                            std::cout << "DEBUG: ";
                            break;
                        case NUClear::INFO:
                            std::cout << "INFO: ";
                            break;
                        case NUClear::WARN:
                            std::cout << "WARN: ";
                            break;
                        case NUClear::ERROR:
                            std::cout << "ERROR: ";
                            break;
                        case NUClear::FATAL:
                            std::cout << "FATAL: ";
                            break;
                    }

                    // Output the message
                    std::cout << message.message << std::endl;
                });
            }

        }  // logging
    }  // support
}  // modules