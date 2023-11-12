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
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct PrimaryTask {};
    struct SecondaryTask {};

    struct SubTask {
        SubTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

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

            // Store the tasks we are given to run
            on<Provide<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) {  //
                events.push_back("simple task 1 from " + t.msg);
            });
            on<Provide<SimpleTask<2>>>().then([this](const SimpleTask<2>& t) {  //
                events.push_back("simple task 2 from " + t.msg);
            });

            on<Provide<PrimaryTask>, SimpleTask<2>, Needs<SubTask>>().then([this]() {
                events.push_back("emitting from primary task");
                emit<Task>(std::make_unique<SimpleTask<2>>("primary task"));
                emit<Task>(std::make_unique<SubTask>("primary task"));
            });

            on<Provide<SubTask>, Needs<SimpleTask<1>>>().then([this](const SubTask& t) {
                events.push_back("emitting from subtask");
                emit<Task>(std::make_unique<SimpleTask<1>>(t.msg));
            });

            on<Provide<SecondaryTask>, Needs<SimpleTask<1>>, Needs<SimpleTask<2>>>().then([this] {
                events.push_back("emitting from secondary task");
                emit<Task>(std::make_unique<SimpleTask<1>>("secondary task"));
                emit<Task>(std::make_unique<SimpleTask<2>>("secondary task"));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting primary task");
                emit<Task>(std::make_unique<PrimaryTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting secondary task");
                emit<Task>(std::make_unique<SecondaryTask>());
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("removing primary task");
                emit<Task>(std::unique_ptr<PrimaryTask>(nullptr));
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
            });
        }
    };

}  // namespace

TEST_CASE("Tests a waiting task can take over subtasks of another task that is being removed",
          "[director][needs][watcher][removal]") {

    NUClear::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting primary task",
        "emitting from primary task",
        "simple task 2 from primary task",
        "emitting from subtask",
        "simple task 1 from primary task",
        "emitting secondary task",
        "removing primary task",
        "emitting from secondary task",
        "simple task 1 from secondary task",
        "simple task 2 from secondary task",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
