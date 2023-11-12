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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#include <catch2/catch_test_macros.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct SimpleTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            a_handle = on<Provide<SimpleTask>>().then([this] {  //
                events.push_back("version a");
            });

            on<Provide<SimpleTask>>().then([this] {  //
                events.push_back("version b");
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting task 1");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("disabling a");
                a_handle.disable();
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting task 2");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("enabling a");
                a_handle.enable();
            });
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                events.push_back("emitting task 3");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
                emit(std::make_unique<Step<5>>());
            });
        }

    private:
        ReactionHandle a_handle;
    };
}  // namespace

TEST_CASE("Test that disabled providers are not considered when choosing a provider", "[director][simple]") {

    NUClear::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting task 1",
        "version a",
        "disabling a",
        "emitting task 2",
        "version b",
        "enabling a",
        "emitting task 3",
        "version a",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
