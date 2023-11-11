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

    struct MainTask {
        explicit MainTask(int subtask_) : subtask(subtask_) {}
        int subtask;
    };

    template <int I>
    struct Subtask {};

    struct CommonDependency {
        explicit CommonDependency(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<MainTask>>().then([this](const MainTask& task) {
                events.push_back("running main task");
                if (task.subtask == 1) {
                    events.push_back("requesting subtask 1");
                    emit<Task>(std::make_unique<Subtask<1>>());
                }
                else if (task.subtask == 2) {
                    events.push_back("requesting subtask 2");
                    emit<Task>(std::make_unique<Subtask<2>>());
                }
            });

            on<Provide<Subtask<1>>, Needs<CommonDependency>>().then([this] {
                events.push_back("running subtask 1");
                emit<Task>(std::make_unique<CommonDependency>("from subtask 1"));
            });

            on<Provide<Subtask<2>>, Needs<CommonDependency>>().then([this] {
                events.push_back("running subtask 2");
                emit<Task>(std::make_unique<CommonDependency>("from subtask 2"));
            });

            on<Provide<CommonDependency>>().then([this](const CommonDependency& t) {  //
                events.push_back("running common dependency from " + t.msg);
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("requesting main task with subtask 1");
                emit<Task>(std::make_unique<MainTask>(1));
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("requesting main task with subtask 2");
                emit<Task>(std::make_unique<MainTask>(2));
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test a provider can replace its task when the new task overlaps in dependencies with the old task",
          "[director][needs][self]") {

    NUClear::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "requesting main task with subtask 1",
        "running main task",
        "requesting subtask 1",
        "running subtask 1",
        "running common dependency from from subtask 1",
        "requesting main task with subtask 2",
        "running main task",
        "requesting subtask 2",
        "running subtask 2",
        "running common dependency from from subtask 2",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
