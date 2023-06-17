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
#include <string>

#include "Director.hpp"
#include "TestBase.hpp"
#include "utility/strutil/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct RunTrigger {};
    struct RemoveTrigger {};
    struct UsesTrigger {};
    struct SimpleTask {};
    struct SubTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        /// Print the subtask state
        std::string decode_run_state(GroupInfo::RunState state) {
            switch (state) {
                case GroupInfo::RunState::NO_TASK: return "NO_TASK";
                case GroupInfo::RunState::RUNNING: return "RUNNING";
                case GroupInfo::RunState::QUEUED: return "QUEUED";
                default: return "ERROR";
            }
        }

        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Trigger<RunTrigger>>().then([this] {
                events.push_back("emitting subtask");
                emit<Task>(std::make_unique<SubTask>());
            });

            on<Trigger<RemoveTrigger>>().then([this] {
                events.push_back("removing subtask");
                emit<Task>(std::unique_ptr<SubTask>(nullptr));
            });

            on<Trigger<UsesTrigger>, Uses<SubTask>>().then([this](const Uses<SubTask>& subtask) {
                events.push_back("root subtask run state: " + decode_run_state(subtask.run_state));
            });

            on<Provide<SimpleTask>, Uses<SubTask>>().then([this](const Uses<SubTask>& subtask) {
                events.push_back("secondary task subtask run state: " + decode_run_state(subtask.run_state));
                events.push_back("emitting subtask");
                emit<Task>(std::make_unique<SubTask>());
            });

            on<Provide<SubTask>>().then([this] { events.push_back("subtask executed"); });

            /**************
             * TEST STEPS *
             **************/
            // Emit the trigger to run the subtask
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting run trigger");
                emit<Scope::DIRECT>(std::make_unique<RunTrigger>());
            });

            // Check the uses state of the subtask
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emit uses trigger");
                emit<Scope::DIRECT>(std::make_unique<UsesTrigger>());
            });

            // Run the simple task, which has a lower priority than the root trigger subtask
            // This should give a queued subtask state on the next run
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task");
                emit<Task>(std::make_unique<SimpleTask>());
            });

            // Run the simple task again to get the queued state
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task again");
                emit<Task>(std::make_unique<SimpleTask>());
            });

            // Remove the root subtask to detect a running subtask on the secondary task
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                events.push_back("emit remove trigger");
                emit<Scope::DIRECT>(std::make_unique<RemoveTrigger>());
            });

            // Run the simple task again to see the running state
            on<Trigger<Step<6>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task again");
                emit<Task>(std::make_unique<SimpleTask>());
            });

            // Emit the uses trigger to check the non-running state
            on<Trigger<Step<7>>, Priority::LOW>().then([this] {
                events.push_back("emit uses trigger");
                emit<Scope::DIRECT>(std::make_unique<UsesTrigger>());
            });

            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
                emit(std::make_unique<Step<5>>());
                emit(std::make_unique<Step<6>>());
                emit(std::make_unique<Step<7>>());
            });
        }

        bool executed = false;
    };
}  // namespace

TEST_CASE("Test that the Uses run state information is correct with root tasks", "[director][uses][state][root]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting run trigger",
        "emitting subtask",
        "subtask executed",
        "emit uses trigger",
        "root subtask run state: RUNNING",
        "emitting simple task",
        "secondary task subtask run state: NO_TASK",
        "emitting subtask",
        "emitting simple task again",
        "secondary task subtask run state: QUEUED",
        "emitting subtask",
        "emit remove trigger",
        "removing subtask",
        "subtask executed",
        "emitting simple task again",
        "secondary task subtask run state: RUNNING",
        "emitting subtask",
        "subtask executed",
        "emit uses trigger",
        "root subtask run state: NO_TASK",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
