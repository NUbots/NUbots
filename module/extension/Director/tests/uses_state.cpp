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

    struct PrimaryTask {
        PrimaryTask(const bool run) : run_subtask(run) {}
        bool run_subtask = false;
    };

    struct SecondaryTask {
        SecondaryTask(const bool run) : run_subtask(run) {}
        bool run_subtask = false;
    };

    struct SubTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<PrimaryTask>, Uses<SubTask>>().then(
                [this](const PrimaryTask& primary, const Uses<SubTask>& subtask) {
                    // Run the subtask if requested
                    if (primary.run_subtask) {
                        events.push_back("emitting subtask");
                        emit<Task>(std::make_unique<SubTask>());
                    }

                    // Check the state of the subtask
                    if (subtask.running) {
                        events.push_back("primary subtask running");
                    }
                    else if (subtask.queued) {
                        events.push_back("primary subtask queued");
                    }
                    else {
                        events.push_back("primary subtask not running");
                    }

                    if (subtask.done) {
                        events.push_back("primary subtask done");
                    }
                    else {
                        events.push_back("primary subtask not done");
                    }
                });

            on<Provide<SecondaryTask>, Uses<SubTask>>().then(
                [this](const SecondaryTask& secondary, const Uses<SubTask>& subtask) {
                    if (secondary.run_subtask) {
                        events.push_back("emitting subtask");
                        emit<Task>(std::make_unique<SubTask>());
                    }

                    if (subtask.running) {
                        events.push_back("secondary subtask running");
                    }
                    else if (subtask.queued) {
                        events.push_back("secondary subtask queued");
                    }
                    else {
                        events.push_back("secondary subtask not running");
                    }

                    if (subtask.done) {
                        events.push_back("secondary subtask done");
                    }
                    else {
                        events.push_back("secondary subtask not done");
                    }
                });

            on<Provide<SubTask>>().then([this] {
                events.push_back("subtask executed");
                emit<Task>(std::make_unique<Done>());
            });

            /**************
             * TEST STEPS *
             **************/
            // Start the primary task without emitting a subtask to get a non-running subtask state
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting primary task");
                emit<Task>(std::make_unique<PrimaryTask>(false), 1);
            });

            // Run the primary task with emitting a subtask to get a running subtask state
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting primary task");
                emit<Task>(std::make_unique<PrimaryTask>(true), 1);
            });

            // Run the primary task again to detect the Done subtask
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting primary task");
                emit<Task>(std::make_unique<PrimaryTask>(true), 1);
            });

            // Run the secondary task with emitting a subtask, with lower priority than the primary task
            // This should give a queued subtask state
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("emitting secondary task");
                emit<Task>(std::make_unique<SecondaryTask>(true));
            });

            // Remove the primary task to detect a running subtask on the secondary task
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                events.push_back("removing primary task");
                emit<Task>(std::unique_ptr<PrimaryTask>(nullptr));
            });

            on<Startup>().then([this] {  //
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
                emit(std::make_unique<Step<5>>());
            });
        }

        bool executed = false;
    };
}  // namespace

TEST_CASE("Test that the Uses state information is correct", "[director][uses][state]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {"emitting primary task",      "primary subtask not running",
                                         "primary subtask not done",   "emitting primary task",
                                         "emitting subtask",           "primary subtask running",
                                         "primary subtask not done",   "subtask executed",
                                         "emitting primary task",      "emitting subtask",
                                         "primary subtask running",    "primary subtask done",
                                         "emitting secondary task",    "secondary subtask not running",
                                         "secondary subtask not done", "removing primary task",
                                         "emitting subtask",           "secondary subtask running",
                                         "secondary subtask not done", "subtask executed"};

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
