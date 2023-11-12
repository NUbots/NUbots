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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#include <catch2/catch_test_macros.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    template <int i>
    struct SimpleTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask<2>>>().then([this] { events.push_back("simple task 2"); });

            /**************
             * TEST STEPS *
             **************/
            // PrimaryTask takes ahold of SubTask and SubSubTask
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task 1");
                emit<Task>(std::make_unique<SimpleTask<1>>());
            });

            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task 2");
                emit<Task>(std::make_unique<SimpleTask<2>>());
            });

            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task 3");
                emit<Task>(std::make_unique<SimpleTask<3>>());
            });

            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test that a Task can be emitted when no Provider exists for it", "[director][provider][missing]") {

    NUClear::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {"emitting simple task 1",
                                         "emitting simple task 2",
                                         "simple task 2",
                                         "emitting simple task 3"};

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
