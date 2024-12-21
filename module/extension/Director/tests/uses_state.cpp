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

    struct PrimaryTask {};
    struct SecondaryTask {};
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


            on<Provide<PrimaryTask>, Uses<SubTask>>().then([this](const Uses<SubTask>& subtask) {
                events.push_back("primary task subtask run state: " + decode_run_state(subtask.run_state));
                events.push_back("emitting subtask");
                emit<Task>(std::make_unique<SubTask>());
            });

            on<Provide<SecondaryTask>, Uses<SubTask>>().then([this](const Uses<SubTask>& subtask) {
                events.push_back("secondary task subtask run state: " + decode_run_state(subtask.run_state));
                events.push_back("emitting subtask");
                emit<Task>(std::make_unique<SubTask>());
            });

            on<Provide<SubTask>>().then([this] { events.push_back("subtask executed"); });

            /**************
             * TEST STEPS *
             **************/
            // Start the primary task, subtask will initially not be running
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting primary task");
                emit<Task>(std::make_unique<PrimaryTask>(), 1);
            });

            // Run the primary task again, subtask will now be running
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting primary task again");
                emit<Task>(std::make_unique<PrimaryTask>(), 1);
            });

            // Run the secondary task with lower priority than the primary task
            // This should give a queued subtask state on the next run
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting secondary task");
                emit<Task>(std::make_unique<SecondaryTask>());
            });

            // Run the secondary task again to get the queued state
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("emitting secondary task again");
                emit<Task>(std::make_unique<SecondaryTask>());
            });

            // Remove the primary task to detect a running subtask on the secondary task
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                events.push_back("removing primary task");
                emit<Task>(std::unique_ptr<PrimaryTask>(nullptr));
            });

            // Run the secondary task again to see the running state
            on<Trigger<Step<6>>, Priority::LOW>().then([this] {
                events.push_back("emitting secondary task again");
                emit<Task>(std::make_unique<SecondaryTask>());
            });

            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
                emit(std::make_unique<Step<5>>());
                emit(std::make_unique<Step<6>>());
            });
        }

        bool executed = false;
    };
}  // namespace

TEST_CASE("Test that the Uses run state information is correct", "[director][uses][state]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting primary task",
        "primary task subtask run state: NO_TASK",
        "emitting subtask",
        "subtask executed",
        "emitting primary task again",
        "primary task subtask run state: RUNNING",
        "emitting subtask",
        "subtask executed",
        "emitting secondary task",
        "secondary task subtask run state: NO_TASK",
        "emitting subtask",
        "emitting secondary task again",
        "secondary task subtask run state: QUEUED",
        "emitting subtask",
        "removing primary task",
        "subtask executed",
        "emitting secondary task again",
        "secondary task subtask run state: RUNNING",
        "emitting subtask",
        "subtask executed",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
