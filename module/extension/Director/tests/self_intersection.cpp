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
#include <fmt/format.h>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct MainTask {};

    template <int I>
    struct SubTask {
        SubTask() : subtask_id(I) {}
        int subtask_id;
    };

    struct Dependency {
        Dependency(const int& subtask_id_) : subtask_id(subtask_id_) {}
        int subtask_id;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<MainTask>>().then([this] {
                // Emit optional tasks with one blocking the other
                events.push_back("emitting task 1");
                emit<Task>(std::make_unique<SubTask<1>>(), 0, true);
                events.push_back("emitting task 2");
                emit<Task>(std::make_unique<SubTask<2>>(), 0, true);
            });

            on<Provide<SubTask<1>>, Needs<Dependency>>().then([this](const SubTask<1>& t) {
                events.push_back(fmt::format("subtask {} executed", t.subtask_id));

                // Emit the dependency task
                events.push_back(fmt::format("emitting dependency from subtask {}", t.subtask_id));
                emit<Task>(std::make_unique<Dependency>(t.subtask_id));
            });

            on<Provide<SubTask<2>>, Needs<Dependency>>().then([this](const SubTask<2>& t) {
                events.push_back(fmt::format("subtask {} executed", t.subtask_id));

                // Emit the dependency task
                events.push_back(fmt::format("emitting dependency from subtask {}", t.subtask_id));
                emit<Task>(std::make_unique<Dependency>(t.subtask_id));
            });

            on<Provide<Dependency>>().then([this](const Dependency& t) {
                events.push_back(fmt::format("dependency from subtask {}", t.subtask_id));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting main task");
                emit<Task>(std::make_unique<MainTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("removing main task");
                emit<Task>(std::unique_ptr<MainTask>(nullptr));
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting main task");
                emit<Task>(std::make_unique<MainTask>());
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("removing main task");
                emit<Task>(std::unique_ptr<MainTask>(nullptr));
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

TEST_CASE("Test that when a task has self intersection it applies priority correctly",
          "[director][priority][optional][self_intersection]") {
    // Run the module
    NUClear::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting main task",
        "emitting task 1",
        "emitting task 2",
        "subtask 1 executed",
        "emitting dependency from subtask 1",
        "dependency from subtask 1",
        "removing main task",
        "emitting main task",
        "emitting task 1",
        "emitting task 2",
        "subtask 1 executed",
        "emitting dependency from subtask 1",
        "dependency from subtask 1",
        "removing main task",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
