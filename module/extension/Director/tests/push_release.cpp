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

    struct SimpleTask {};
    struct Helper {};

    struct Condition {
        enum Value { BLOCK, ALLOW } value;
        Condition(const Value& v) : value(v) {}
        operator int() const {
            return value;
        }
        operator std::string() const {
            switch (value) {
                case BLOCK: return "BLOCK";
                case ALLOW: return "ALLOW";
                default: return "UNKNOWN";
            }
        }
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 6> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            on<Provide<SimpleTask>, When<Condition, std::equal_to, Condition::ALLOW>>().then([this] {  //
                events.push_back("task executed");
            });
            on<Stop<SimpleTask>>().then([this] {  //
                events.push_back("task stopped");
            });

            on<Provide<Helper>>().then([this] {  //
                events.push_back("helper waiting");
            });
            on<Provide<Helper>, Causing<Condition, Condition::ALLOW>>().then([this] {
                events.push_back("helper causing allow");
                emit(std::make_unique<Condition>(Condition::ALLOW));
            });
            on<Stop<Helper>>().then([this] {  //
                events.push_back("helper stopped");
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
                events.push_back("emitting task");
                emit<Task>(std::make_unique<SimpleTask>(), 100);
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                // Remove the pushed group's own task while it is being pushed.
                // The push cannot function without a running group so it must be released.
                events.push_back("removing helper");
                emit<Task>(std::unique_ptr<Helper>(nullptr));
            });
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                // With the helper gone nothing maintains the condition, so blocking it stops the task
                events.push_back("emitting blocked condition again");
                emit(std::make_unique<Condition>(Condition::BLOCK));
            });
            on<Trigger<Step<6>>, Priority::LOW>().then([this] {
                // A new helper task must run on its preferred provider.
                // If the old push was not released the stale push would force it onto the causing provider.
                events.push_back("emitting helper task again");
                emit<Task>(std::make_unique<Helper>(), 10);
            });
        }
    };
}  // namespace

TEST_CASE("Test that a push is released when the pushed group's task is removed", "[director][causing][push]") {

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
        "emitting task",
        "helper causing allow",
        "task executed",
        "removing helper",
        "helper stopped",
        "emitting blocked condition again",
        "task stopped",
        "emitting helper task again",
        "helper waiting",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
