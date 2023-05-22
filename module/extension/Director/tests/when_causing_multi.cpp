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

#include <catch.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct SimpleTask {};

    template <int i>
    struct Subtask {};

    template <int i>
    struct Helper {};

    template <int i>
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

            on<Provide<SimpleTask>, Needs<Subtask<1>>, Needs<Subtask<2>>>().then([this] {
                events.push_back("task executed");
                emit<Task>(std::make_unique<Subtask<1>>());
                emit<Task>(std::make_unique<Subtask<2>>());
            });


            on<Provide<Subtask<1>>, When<Condition<1>, std::equal_to, Condition<1>::ALLOW>>().then([this] {  //
                events.push_back("subtask 1 executed");
            });
            on<Provide<Subtask<2>>, When<Condition<2>, std::equal_to, Condition<2>::ALLOW>>().then([this] {  //
                events.push_back("subtask 1 executed");
            });


            on<Provide<Helper<1>>>().then([this] { events.push_back("helper 1 waiting"); });
            on<Provide<Helper<1>>, Causing<Condition<1>, Condition<1>::ALLOW>>().then([this] {
                events.push_back("helper 1 causing allow 1");
                emit(std::make_unique<Condition<1>>(Condition<1>::ALLOW));
            });

            on<Provide<Helper<2>>>().then([this] { events.push_back("helper 2 waiting"); });
            on<Provide<Helper<2>>, Causing<Condition<2>, Condition<2>::ALLOW>>().then([this] {
                events.push_back("helper 2 causing allow 2");
                emit(std::make_unique<Condition<2>>(Condition<2>::ALLOW));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Start up the helper
                events.push_back("emitting helper tasks");
                emit<Task>(std::make_unique<Helper<1>>(), 10);
                emit<Task>(std::make_unique<Helper<2>>(), 10);
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emitting a blocked condition
                events.push_back("emitting blocked conditions");
                emit(std::make_unique<Condition<1>>(Condition<1>::BLOCK));
                emit(std::make_unique<Condition<2>>(Condition<2>::BLOCK));
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

TEST_CASE("Test that if multiple things that are needed have when+causings all will run", "[director][!mayfail]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting helper tasks",
        "helper 1 waiting",
        "helper 2 waiting",
        "emitting blocked conditions",
        "emitting task at low priority",
        "emitting task at high priority",
        "helper 1 causing allow 1",
        "helper 2 causing allow 2",
        "task executed",
        "subtask 1 executed",
        "subtask 2 executed",
        "helper 1 waiting",
        "helper 2 waiting",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
