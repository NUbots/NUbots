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
    struct Helper {};

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

            on<Provide<Helper>>().then([this] {
                // Task has been executed!
                events.push_back("helper waiting");
            });
            on<Provide<Helper>, Causing<Condition, Condition::ALLOW>>().then([this] {
                // Task has been executed!
                events.push_back("helper causing allow");
                emit(std::make_unique<Condition>(Condition::ALLOW));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Start up the helper
                events.push_back("emitting helper task");
                emit<Task>(std::make_unique<Helper>(), 10);
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emitting a blocked condition
                events.push_back("emitting blocked condition");
                emit(std::make_unique<Condition>(Condition::BLOCK));
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                // Emit the task at a lower priority than the helper and it shouldn't be able to push it and be blocked
                events.push_back("emitting task at low priority");
                emit<Task>(std::make_unique<SimpleTask>(), 1);
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                // Increase the priority of the task so it can push the helper
                events.push_back("emitting task at high priority");
                emit<Task>(std::make_unique<SimpleTask>(), 100);
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
            });
        }
    };
}  // namespace

TEST_CASE("Test that the causing keyword can provide what another module needs", "[director][!mayfail]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting helper task",
        "helper waiting",
        "emitting blocked condition",
        "emitting task at low priority",
        "emitting task at high priority",
        "helper causing allow",
        "task executed",
        "helper waiting",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
