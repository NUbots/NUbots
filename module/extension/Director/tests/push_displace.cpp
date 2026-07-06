/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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

    struct TaskA {};
    struct TaskB {};
    struct Helper {};

    struct Condition {
        enum Value { BLOCK, ALLOW_A, ALLOW_B } value;
        Condition(const Value& v) : value(v) {}
        operator int() const {
            return value;
        }
        operator std::string() const {
            switch (value) {
                case BLOCK: return "BLOCK";
                case ALLOW_A: return "ALLOW_A";
                case ALLOW_B: return "ALLOW_B";
                default: return "UNKNOWN";
            }
        }
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 7> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            on<Provide<TaskA>, When<Condition, std::equal_to, Condition::ALLOW_A>>().then([this] {  //
                events.push_back("task a executed");
            });
            on<Stop<TaskA>>().then([this] {  //
                events.push_back("task a stopped");
            });

            on<Provide<TaskB>, When<Condition, std::equal_to, Condition::ALLOW_B>>().then([this] {  //
                events.push_back("task b executed");
            });

            on<Provide<Helper>>().then([this] {  //
                events.push_back("helper waiting");
            });
            on<Provide<Helper>, Causing<Condition, Condition::ALLOW_A>>().then([this] {
                events.push_back("helper causing allow a");
                emit(std::make_unique<Condition>(Condition::ALLOW_A));
            });
            on<Provide<Helper>, Causing<Condition, Condition::ALLOW_B>>().then([this] {
                events.push_back("helper causing allow b");
                emit(std::make_unique<Condition>(Condition::ALLOW_B));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting helper task");
                emit<Task>(std::make_unique<Helper>(), 10);
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting blocked condition");
                emit(std::make_unique<Condition>(Condition::BLOCK));
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting task a");
                emit<Task>(std::make_unique<TaskA>(), 50);
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                // A higher priority pusher takes the push over, blocking task a
                events.push_back("emitting task b");
                emit<Task>(std::make_unique<TaskB>(), 100);
            });
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                // Removing task b releases the push and the queued task a should push again
                events.push_back("removing task b");
                emit<Task>(std::unique_ptr<TaskB>(nullptr));
            });
            on<Trigger<Step<6>>, Priority::LOW>().then([this] {
                // Removing task a releases its push too and the helper reverts to its preferred provider
                events.push_back("removing task a");
                emit<Task>(std::unique_ptr<TaskA>(nullptr));
            });
            on<Trigger<Step<7>>, Priority::LOW>().then([this] {
                // End the test with nothing running so shutdown doesn't produce teardown events
                events.push_back("removing helper");
                emit<Task>(std::unique_ptr<Helper>(nullptr));
            });
        }
    };
}  // namespace

TEST_CASE("Test that a higher priority pusher displaces a lower priority one and gives it back when removed",
          "[director][causing][push]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting helper task",
        "helper waiting",
        "emitting blocked condition",
        "emitting task a",
        "helper causing allow a",
        "task a executed",
        "emitting task b",
        "helper causing allow b",
        "task a stopped",
        "task b executed",
        "removing task b",
        "helper causing allow a",
        "task a executed",
        "removing task a",
        "helper waiting",
        "task a stopped",
        "removing helper",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
