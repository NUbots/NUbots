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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

// Uncomment this line when other test files are added
//#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file

#include <catch.hpp>
#include "motion"

/*namespace {
    struct SimpleMessage {
        int data;
    };

    class TestMotion : public NUClear::Reactor {
    public:
        TestReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<SimpleMessage>>().then([this](const SimpleMessage& message) {

                // The message we received should have test == 10
                REQUIRE(message.data == 10);

                // We are finished the test
                this->powerplant.shutdown();
            });
        }
    };
}


TEST_CASE("A very basic test for Emit and On", "[api][trigger]") {

    NUClear::PowerPlant::Configuration config;
    config.threadCount = 1;
    NUClear::PowerPlant plant(config);
    plant.install<TestReactor>();

    auto message = std::make_unique<SimpleMessage>(SimpleMessage { 10 });

    plant.emit(message);

    plant.start();
}*/
