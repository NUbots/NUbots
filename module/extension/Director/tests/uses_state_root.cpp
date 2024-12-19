/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <catch2/catch_test_macros.hpp>
#include <nuclear>
#include <string>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct RunTrigger {};
    struct RemoveTrigger {};
    struct UsesTrigger {};
    struct SimpleTask {};
    struct SubTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 7> {
    public:
        /// Print the subtask state
        std::string decode_run_state(RunState state) {
            switch (state) {
                case RunState::NO_TASK: return "NO_TASK";
                case RunState::RUNNING: return "RUNNING";
                case RunState::QUEUED: return "QUEUED";
                default: return "ERROR";
            }
        }

        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

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
                emit<Scope::INLINE>(std::make_unique<RunTrigger>());
            });

            // Check the uses state of the subtask
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emit uses trigger");
                emit<Scope::INLINE>(std::make_unique<UsesTrigger>());
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
                emit<Scope::INLINE>(std::make_unique<RemoveTrigger>());
            });

            // Run the simple task again to see the running state
            on<Trigger<Step<6>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task again");
                emit<Task>(std::make_unique<SimpleTask>());
            });

            // Emit the uses trigger to check the non-running state
            on<Trigger<Step<7>>, Priority::LOW>().then([this] {
                events.push_back("emit uses trigger");
                emit<Scope::INLINE>(std::make_unique<UsesTrigger>());
            });
        }

        bool executed = false;
    };
}  // namespace

TEST_CASE("Test that the Uses run state information is correct with root tasks", "[director][uses][state][root]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
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
