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
#include "utility/strutil/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct SimpleTask {};

    struct Condition {
        enum Value { BLOCK, ALLOW } value;
        Condition(const Value& v) : value(v) {}
        operator int() const {
            return value;
        }
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask>, When<Condition, std::equal_to, Condition::ALLOW>>().then([this] {
                // Task has been executed!
                events.push_back("task executed");
            });

            on<Start<SimpleTask>>().then([this] {
                // Task has been started!
                events.push_back("task started");
            });
            on<Stop<SimpleTask>>().then([this] {
                // Task has been stopped!
                events.push_back("task stopped");
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // At this point condition hasn't been emitted, so this task should be blocked by default
                events.push_back("emitting task #1");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emitting a blocked condition
                events.push_back("emitting blocked condition #1");
                emit(std::make_unique<Condition>(Condition::BLOCK));
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                // Emitting the task again to ensure it's not executed
                events.push_back("emitting task #2");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                // This should make it run since it's now allowed
                events.push_back("emitting allowed condition");
                emit(std::make_unique<Condition>(Condition::ALLOW));
            });
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                // This task should run fine since it's allowed
                events.push_back("emitting task #3");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<6>>, Priority::LOW>().then([this] {
                // This should stop the running task
                events.push_back("emitting blocked condition #2");
                emit(std::make_unique<Condition>(Condition::BLOCK));
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
                emit(std::make_unique<Step<5>>());
                emit(std::make_unique<Step<6>>());
            });
        }
    };
}  // namespace

TEST_CASE("Test that the when keyword blocks and allows running as expected", "[director][when][blocking]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting task #1",
        "emitting blocked condition #1",
        "emitting task #2",
        "emitting allowed condition",
        "task started",
        "task executed",
        "emitting task #3",
        "task executed",
        "emitting blocked condition #2",
        "task stopped",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
