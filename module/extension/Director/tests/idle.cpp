/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

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
    NUClear::Configuration config;
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
