/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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

#include <catch2/catch_test_macros.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

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

    class TestReactor : public TestBase<TestReactor, 6> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

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
        }
    };
}  // namespace

TEST_CASE("Test that the when keyword blocks and allows running as expected", "[director][when][blocking]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
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
