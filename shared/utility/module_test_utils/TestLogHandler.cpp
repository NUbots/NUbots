/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#define CATCH_CONFIG_RUNNER
#include "TestLogHandler.hpp"

#include <catch.hpp>
#include <iostream>
#include <memory>
#include <nuclear>
#include <sstream>
#include <string>

#include "utility/strutil/ansi.hpp"

utility::module_test::TestLogHandler::TestLogHandler(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

    using NUClear::message::LogMessage;
    using utility::strutil::Colour;

    on<Trigger<LogMessage>>().then([this](const LogMessage& message) {
        std::lock_guard<std::mutex> lock(mutex);

        // Where this message came from
        std::string source{};

        // If we know where this log message came from, we display that
        if (message.task != nullptr) {
            std::cout << "Inside the block" << std::endl;
            // Get our reactor name
            std::string reactor = message.task->identifier[1];

            // Strip to the last colon if we have one
            const size_t last_colon = reactor.find_last_of(':');
            reactor                 = last_colon == std::string::npos ? reactor : reactor.substr(last_colon + 1);

            // This is our source
            source =
                reactor + " " + (message.task->identifier[0].empty() ? "" : "- " + message.task->identifier[0] + " ");
            std::cout << "At the end of the block" << std::endl;
        }

        std::cout << "After the block" << std::endl;

        // Output the level
        std::stringstream log_message;
        log_message << source;
        switch (message.level) {
            case NUClear::TRACE: log_message << "TRACE: "; break;
            case NUClear::DEBUG: log_message << Colour::green << "DEBUG: "; break;
            case NUClear::INFO: log_message << Colour::brightblue << "INFO: "; break;
            case NUClear::WARN: log_message << Colour::yellow << "WARN: "; break;
            case NUClear::ERROR: log_message << Colour::brightred << "(╯°□°）╯︵ ┻━┻: "; break;
            case NUClear::FATAL: log_message << Colour::brightred << "(ノಠ益ಠ)ノ彡┻━┻: "; break;
        }

        // Output the message
        UNSCOPED_INFO(log_message.str() << message.message);
    });
}
