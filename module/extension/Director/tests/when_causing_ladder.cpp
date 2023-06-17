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

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
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
