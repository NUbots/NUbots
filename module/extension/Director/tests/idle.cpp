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
#include "utility/strutil/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct DependentTask {
        DependentTask(const int& id_) : id(id_) {}
        int id;
    };
    struct SimpleTask {
        SimpleTask(const int& id_) : id(id_) {}
        int id;
    };
    struct TriggerTest {};


    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask>>().then([this](const SimpleTask& t) {
                events.push_back("task " + std::to_string(t.id));

                // First time emit two dependent tasks
                if (t.id == 1) {
                    events.push_back("emitting initial dependent task");
                    emit<Task>(std::make_unique<DependentTask>(t.id));
                }

                // Second time emit idle
                else if (t.id == 2) {
                    events.push_back("emitting idle");
                    emit<Task>(std::make_unique<Idle>());
                }

                // Third time emit no tasks
                else if (t.id == 3) {
                    events.push_back("emitting no tasks");
                }

                // Fourth time emit two dependent tasks
                else if (t.id == 4) {
                    events.push_back("emitting final dependent task");
                    emit<Task>(std::make_unique<DependentTask>(t.id));
                }
            });

            // These will alternate on and off depending on the state of simple task
            // Some of the time the providers won't be active and shouldn't be able to run
            on<Provide<DependentTask>, Trigger<TriggerTest>>().then([this](const DependentTask& d) {  //
                events.push_back("dependent run with " + std::to_string(d.id));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting trigger 0");
                emit(std::make_unique<TriggerTest>());
                events.push_back("emitting task 1");
                emit<Task>(std::make_unique<SimpleTask>(1));
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting trigger 1");
                emit(std::make_unique<TriggerTest>());
            });

            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting task 2");
                emit<Task>(std::make_unique<SimpleTask>(2));
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("emitting trigger 2");
                emit(std::make_unique<TriggerTest>());
            });

            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                events.push_back("emitting task 3");
                emit<Task>(std::make_unique<SimpleTask>(3));
            });
            on<Trigger<Step<6>>, Priority::LOW>().then([this] {
                events.push_back("emitting trigger 3");
                emit(std::make_unique<TriggerTest>());
            });

            on<Trigger<Step<7>>, Priority::LOW>().then([this] {
                events.push_back("emitting task 4");
                emit<Task>(std::make_unique<SimpleTask>(4));
            });
            on<Trigger<Step<8>>, Priority::LOW>().then([this] {
                events.push_back("emitting trigger 4");
                emit(std::make_unique<TriggerTest>());
            });

            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
                emit(std::make_unique<Step<5>>());
                emit(std::make_unique<Step<6>>());
                emit(std::make_unique<Step<7>>());
                emit(std::make_unique<Step<8>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test that when Idle is emitted nothing changes with subtasks", "[director][triggered][idle]") {

    // Run the module
    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting trigger 0",
        "emitting task 1",
        "task 1",
        "emitting initial dependent task",
        "dependent run with 1",
        "emitting trigger 1",
        "dependent run with 1",
        "emitting task 2",
        "task 2",
        "emitting idle",
        "emitting trigger 2",
        "dependent run with 1",
        "emitting task 3",
        "task 3",
        "emitting no tasks",
        "emitting trigger 3",
        "emitting task 4",
        "task 4",
        "emitting final dependent task",
        "dependent run with 4",
        "emitting trigger 4",
        "dependent run with 4",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
