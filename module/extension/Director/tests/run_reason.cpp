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

    struct SimpleTask {};
    struct Subtask {};
    struct TriggerTest {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        std::string decode_reason(const RunInfo::RunReason& reason) {
            switch (reason) {
                case RunInfo::RunReason::OTHER_TRIGGER: return "OTHER_TRIGGER"; break;
                case RunInfo::RunReason::NEW_TASK: return "NEW_TASK"; break;
                case RunInfo::RunReason::STARTED: return "STARTED"; break;
                case RunInfo::RunReason::STOPPED: return "STOPPED"; break;
                case RunInfo::RunReason::SUBTASK_DONE: return "SUBTASK_DONE"; break;
                case RunInfo::RunReason::PUSHED: return "PUSHED"; break;
                default: return "ERROR"; break;
            };
        }

        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask>, Trigger<TriggerTest>>().then([this](const RunInfo& info) {
                events.push_back("simple task ran because: " + decode_reason(info.run_reason));

                // If we get a new task then emit a subtask
                // When we re-run because of SUBTASK_DONE we won't re-emit this subtask making it stop
                if (info.run_reason == RunInfo::RunReason::NEW_TASK) {
                    events.push_back("emitting subtask");
                    emit<Task>(std::make_unique<Subtask>());
                }

                if (info.run_reason == RunInfo::RunReason::OTHER_TRIGGER) {
                    events.push_back("emitting simple task done");
                    emit<Task>(std::make_unique<Done>());
                }
            });

            on<Start<SimpleTask>>().then([this](const RunInfo& info) {  //
                events.push_back("simple task started because: " + decode_reason(info.run_reason));
            });
            on<Stop<SimpleTask>>().then([this](const RunInfo& info) {  //
                events.push_back("simple task stopped because: " + decode_reason(info.run_reason));
            });

            // This subtask will immediately return done
            on<Provide<Subtask>, Trigger<TriggerTest>>().then([this](const RunInfo& info) {
                events.push_back("subtask ran because: " + decode_reason(info.run_reason));
                events.push_back("emitting subtask done");
                emit<Task>(std::make_unique<Done>());
            });

            on<Start<Subtask>>().then([this](const RunInfo& info) {  //
                events.push_back("subtask started because: " + decode_reason(info.run_reason));
            });
            on<Stop<Subtask>>().then([this](const RunInfo& info) {  //
                events.push_back("subtask stopped because: " + decode_reason(info.run_reason));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Emit initial trigger test so we can run the task
                emit(std::make_unique<TriggerTest>());
                // Checks STARTED, NEW_TASK, SUBTASK_DONE and STOPPED for subtask
                events.push_back("emitting simple task");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Checks OTHER_TRIGGER and STOPPED for simple task
                events.push_back("emitting trigger test");
                emit(std::make_unique<TriggerTest>());
            });

            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
            });
        }
    };

}  // namespace

TEST_CASE("Tests that the reason for a provider being executed can be provided in its RunInfo",
          "[director][triggered][runinfo][runreason]") {

    // Run the module
    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting simple task",
        "simple task started because: STARTED",
        "simple task ran because: NEW_TASK",
        "emitting subtask",
        "subtask started because: STARTED",
        "subtask ran because: NEW_TASK",
        "emitting subtask done",
        "simple task ran because: SUBTASK_DONE",
        "subtask stopped because: STOPPED",
        "emitting trigger test",
        "simple task ran because: OTHER_TRIGGER",
        "emitting simple task done",
        "simple task stopped because: STOPPED",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
