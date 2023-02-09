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

    template <int i>
    struct SimpleTask {
        SimpleTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask<0>>>().then(
                [this](const SimpleTask<0>& t) { emit<Task>(std::make_unique<SimpleTask<1>>(t.msg)); });

            on<Provide<SimpleTask<1>>>().then(
                [this](const SimpleTask<1>& t) { emit<Task>(std::make_unique<SimpleTask<2>>(t.msg)); });

            on<Provide<SimpleTask<2>>>().then([this](const SimpleTask<2>& t) { events.push_back(t.msg); });

            on<Provide<SimpleTask<3>>, Needs<SimpleTask<1>>>().then(
                [this](const SimpleTask<3>& t) { emit<Task>(std::make_unique<SimpleTask<1>>(t.msg)); });

            /**************
             * TEST STEPS *
             **************/
            // SimpleTask<0> takes ahold of SimpleTask<1> and SimpleTask<2>
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting first task");
                emit<Task>(std::make_unique<SimpleTask<0>>("first task"), 1);
            });

            // SimpleTask<3> needs SimpleTask<1>, so it will watch
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting watcher");
                emit<Task>(std::make_unique<SimpleTask<3>>("watcher task"), 0);
            });

            // Remove SimpleTask<0> so SimpleTask<3> can take over
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("removing first task");
                emit<Task>(std::unique_ptr<SimpleTask<0>>(nullptr));
            });

            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test that a watcher can take over from another provider", "[director][remove][watcher]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {"emitting first task",
                                         "first task",
                                         "emitting watcher",
                                         "removing first task",
                                         "watcher task"};

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
