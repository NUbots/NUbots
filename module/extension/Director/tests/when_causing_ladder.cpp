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
        enum Value { LEVEL_0, LEVEL_1, LEVEL_2, LEVEL_3, LEVEL_4 } value;
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

            on<Provide<SimpleTask>, When<Condition, std::equal_to, Condition::LEVEL_4>>().then([this] {  //
                events.push_back("task executed");
            });

            on<Provide<Helper>>().then([this] {  //
                events.push_back("helper waiting");
            });
            on<Provide<Helper>,
               Causing<Condition, Condition::LEVEL_4>,
               When<Condition, std::equal_to, Condition::LEVEL_3>>()
                .then([this] {
                    events.push_back("helper causing level 4");
                    emit(std::make_unique<Condition>(Condition::LEVEL_4));
                });
            on<Provide<Helper>,
               Causing<Condition, Condition::LEVEL_3>,
               When<Condition, std::equal_to, Condition::LEVEL_2>>()
                .then([this] {
                    events.push_back("helper causing level 3");
                    emit(std::make_unique<Condition>(Condition::LEVEL_3));
                });
            on<Provide<Helper>,
               Causing<Condition, Condition::LEVEL_2>,
               When<Condition, std::equal_to, Condition::LEVEL_1>>()
                .then([this] {
                    events.push_back("helper causing level 2");
                    emit(std::make_unique<Condition>(Condition::LEVEL_2));
                });
            on<Provide<Helper>, Causing<Condition, Condition::LEVEL_1>>().then([this] {
                events.push_back("helper causing level 1");
                emit(std::make_unique<Condition>(Condition::LEVEL_1));
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
                // Emit the task
                events.push_back("emitting task");
                emit<Task>(std::make_unique<SimpleTask>(), 50);
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
            });
        }
    };
}  // namespace

TEST_CASE("Test that when/causing relationships can be cascaded", "[director][!mayfail]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting helper task",
        "helper waiting",
        "emitting task",
        "helper causing level 1",
        "helper causing level 2",
        "helper causing level 3",
        "helper causing level 4",
        "task executed",
        "helper waiting",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
