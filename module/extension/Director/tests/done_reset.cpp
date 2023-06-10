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
    struct SubtaskA {};
    struct SubtaskB {};
    struct PokeA {};
    struct PokeB {};

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

            on<Provide<SimpleTask>, Uses<SubtaskA>, Uses<SubtaskB>>().then(
                [this](const RunInfo& info, const Uses<SubtaskA>& a, const Uses<SubtaskB>& b) {
                    events.push_back("SimpleTask " + decode_reason(info.run_reason)
                                     + " a.done: " + std::to_string(a.done) + " b.done: " + std::to_string(b.done));

                    // Emit tasks if new task happens
                    if (info.run_reason == RunInfo::RunReason::NEW_TASK) {
                        events.push_back("emitting SubtaskA and SubtaskB");
                        emit<Task>(std::make_unique<SubtaskA>());
                        emit<Task>(std::make_unique<SubtaskB>());
                    }
                    else {
                        emit<Task>(std::make_unique<Idle>());
                    }
                });

            on<Provide<SubtaskA>, Trigger<PokeA>>().then([this](const RunInfo& info) {
                if (info.run_reason == RunInfo::RunReason::OTHER_TRIGGER) {
                    events.push_back("SubtaskA done");
                    emit<Task>(std::make_unique<Done>());
                }
                else {
                    events.push_back("ran SubtaskA");
                }
            });

            on<Provide<SubtaskB>, Trigger<PokeB>>().then([this](const RunInfo& info) {
                if (info.run_reason == RunInfo::RunReason::OTHER_TRIGGER) {
                    events.push_back("SubtaskB done");
                    emit<Task>(std::make_unique<Done>());
                }
                else {
                    events.push_back("ran SubtaskB");
                }
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Emitting initial pokes so subtasks can run
                emit(std::make_unique<PokeA>());
                emit(std::make_unique<PokeB>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting PokeA");
                emit(std::make_unique<PokeA>());
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<6>>, Priority::LOW>().then([this] {
                events.push_back("emitting PokeB");
                emit(std::make_unique<PokeB>());
            });
            on<Trigger<Step<7>>, Priority::LOW>().then([this] {
                events.push_back("emitting PokeA");
                emit(std::make_unique<PokeA>());
            });
            on<Trigger<Step<8>>, Priority::LOW>().then([this] {
                events.push_back("removing simple task");
                emit<Task>(std::unique_ptr<SimpleTask>(nullptr));
            });

            // Repeat steps
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
                emit(std::make_unique<Step<5>>());
                emit(std::make_unique<Step<6>>());
                emit(std::make_unique<Step<7>>());
                emit(std::make_unique<Step<8>>());

                // Repeat steps 2-7 to test again
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
                emit(std::make_unique<Step<5>>());
                emit(std::make_unique<Step<6>>());
                emit(std::make_unique<Step<7>>());
                emit(std::make_unique<Step<8>>());
            });
        }

        bool executed = false;
    };
}  // namespace

TEST_CASE("Test that task done resets when given a new task to execute", "[director][done][reset]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting simple task",
        "SimpleTask NEW_TASK a.done: 0 b.done: 0",
        "emitting SubtaskA and SubtaskB",
        "ran SubtaskA",
        "ran SubtaskB",
        "emitting PokeA",
        "SubtaskA done",
        "SimpleTask SUBTASK_DONE a.done: 1 b.done: 0",
        "emitting simple task",
        "SimpleTask NEW_TASK a.done: 1 b.done: 0",
        "emitting SubtaskA and SubtaskB",
        "ran SubtaskA",
        "ran SubtaskB",
        "emitting simple task",
        "SimpleTask NEW_TASK a.done: 0 b.done: 0",
        "emitting SubtaskA and SubtaskB",
        "ran SubtaskA",
        "ran SubtaskB",
        "emitting PokeB",
        "SubtaskB done",
        "SimpleTask SUBTASK_DONE a.done: 0 b.done: 1",
        "emitting PokeA",
        "SubtaskA done",
        "SimpleTask SUBTASK_DONE a.done: 1 b.done: 1",
        "removing simple task",
        "emitting simple task",
        "SimpleTask NEW_TASK a.done: 0 b.done: 0",
        "emitting SubtaskA and SubtaskB",
        "ran SubtaskA",
        "ran SubtaskB",
        "emitting PokeA",
        "SubtaskA done",
        "SimpleTask SUBTASK_DONE a.done: 1 b.done: 0",
        "emitting simple task",
        "SimpleTask NEW_TASK a.done: 1 b.done: 0",
        "emitting SubtaskA and SubtaskB",
        "ran SubtaskA",
        "ran SubtaskB",
        "emitting simple task",
        "SimpleTask NEW_TASK a.done: 0 b.done: 0",
        "emitting SubtaskA and SubtaskB",
        "ran SubtaskA",
        "ran SubtaskB",
        "emitting PokeB",
        "SubtaskB done",
        "SimpleTask SUBTASK_DONE a.done: 0 b.done: 1",
        "emitting PokeA",
        "SubtaskA done",
        "SimpleTask SUBTASK_DONE a.done: 1 b.done: 1",
        "removing simple task",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
