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

#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <iostream>
#include <memory>
#include <nuclear>
#include <sstream>
#include <string>

#include "KinematicsConfiguration.hpp"

#include "extension/FileWatcher/src/FileWatcher.hpp"

#include "message/motion/KinematicsModel.hpp"

#include "utility/strutil/ansi.hpp"

namespace {
    using message::motion::KinematicsModel;
    using NUClear::message::LogMessage;
    using utility::strutil::Colour;

    class TestLogHandler : public NUClear::Reactor {
    public:
        TestLogHandler(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<LogMessage>>().then([this](const LogMessage& message) {
                std::lock_guard<std::mutex> lock(mutex);

                // Where this message came from
                std::string source = "";

                // If we know where this log message came from, we display that
                if (message.task != nullptr) {
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

    private:
        std::mutex mutex;
    };


    std::unique_ptr<KinematicsModel> saved_model = nullptr;

    class TestReactor : public NUClear::Reactor {
    public:
        TestReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Startup>().then([this] { emit<Scope::WATCHDOG>(ServiceWatchdog<TestReactor>()); });

            // Auto stop the test after 10 seconds
            on<Watchdog<TestReactor, 10, std::chrono::seconds>>().then([this] {
                log<NUClear::INFO>("No KinematicsModel message received in 10 seconds. Aborting.");
                powerplant.shutdown();
            });

            // Trigger on the KinematicsModel message
            on<Trigger<KinematicsModel>>().then([this](const KinematicsModel& model) {
                log<NUClear::INFO>("Received a KinematicsModel message. Test is now over.");
                saved_model = std::make_unique<KinematicsModel>(model);

                // When we receive the KinematicsModel message we can shutdown
                powerplant.shutdown();
            });
        }
    };
}  // namespace

TEST_CASE("Testing the Kinematics Configuration module", "[module][motion][KinematicsConfiguration]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 2;
    NUClear::PowerPlant plant(config);

    INFO("Installing TestLogHandler");
    plant.install<TestLogHandler>();
    INFO("Installing extension::FileWatcher");
    plant.install<module::extension::FileWatcher>();
    INFO("Installing TestReactor");
    plant.install<TestReactor>();
    INFO("Installing motion::KinematicsConfiguration");
    plant.install<module::motion::KinematicsConfiguration>();

    INFO("Starting PowerPlant");
    plant.start();

    // Require that a model was saved
    REQUIRE(saved_model != nullptr);

    // Now check values in model to ensure correctness
    REQUIRE(saved_model->head.INTERPUPILLARY_DISTANCE == 0.068f);
}
