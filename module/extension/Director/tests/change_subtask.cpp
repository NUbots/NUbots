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

    struct ParentTask {
        ParentTask(bool subtask) : subtask_a(subtask) {}
        bool subtask_a = false;
    };

    template <int N>
    struct SubTask {
        SubTask(const std::string& msg) : msg(msg) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<ParentTask>>().then([this](const ParentTask& task) {
                if (task.subtask_a) {
                    events.push_back("parent task with subtask 1");
                    emit<Task>(std::make_unique<SubTask<1>>("from parent task"));
                }
                else {
                    events.push_back("parent task with subtask 2");
                    emit<Task>(std::make_unique<SubTask<2>>("from parent task"));
                }
            });

            on<Provide<SubTask<1>>>().then([this](const SubTask<1>& t) {  //
                events.push_back("subtask 1 " + t.msg);
                emit<Task>(std::make_unique<SubTask<2>>("from subtask 1"));
            });

            on<Provide<SubTask<2>>>().then([this](const SubTask<2>& t) {  //
                events.push_back("subtask 2 " + t.msg);
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting parent task with subtask 1");
                emit<Task>(std::make_unique<ParentTask>(true));
            });

            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting parent task with subtask 2");
                emit<Task>(std::make_unique<ParentTask>(false));
            });

            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
            });
        }

    private:
        ReactionHandle a_handle;
    };
}  // namespace

TEST_CASE("Test that subtasks can be changed", "[director][subtasks][change]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting parent task with subtask 1",
        "parent task with subtask 1",
        "subtask 1 from parent task",
        "subtask 2 from subtask 1",
        "emitting parent task with subtask 2",
        "parent task with subtask 2",
        "subtask 2 from parent task",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
