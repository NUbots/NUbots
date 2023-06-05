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

    struct SimpleTask {};
    struct TriggerTest {};
    template <int i>
    struct DependentTask {
        DependentTask(const int& id_) : id(id_) {}
        int id;
    };
    struct OtherData {
        OtherData(const int& id_) : id(id_) {}
        int id;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask>, Trigger<OtherData>>().then([this](const OtherData& d) {
                events.push_back("task " + std::to_string(d.id));

                if (d.id % 2 == 0) {
                    events.push_back("emitting dependent task 1");
                    emit<Task>(std::make_unique<DependentTask<1>>(d.id));
                }
                else {
                    events.push_back("emitting dependent task 2");
                    emit<Task>(std::make_unique<DependentTask<2>>(d.id));
                }
            });

            // These will alternate on and off depending on the state of simple task
            // Some of the time the providers won't be active and shouldn't be able to run
            on<Provide<DependentTask<1>>, Trigger<TriggerTest>>().then([this](const DependentTask<1>& d) {  //
                events.push_back("A: " + std::to_string(d.id));
            });
            on<Provide<DependentTask<2>>, Trigger<TriggerTest>>().then([this](const DependentTask<2>& d) {  //
                events.push_back("B: " + std::to_string(d.id));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Emit inital data so it's in the cache and the triggers can run
                events.push_back("emitting data 0");
                emit(std::make_unique<OtherData>(0));
                events.push_back("emitting Trigger Test");
                emit(std::make_unique<TriggerTest>());

                // Emit the initial simple task
                events.push_back("emitting simple task");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emit a trigger test to see if the dependent tasks are blocked
                events.push_back("emitting Trigger Test");
                emit(std::make_unique<TriggerTest>());
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                // Emit another data
                events.push_back("emitting data 1");
                emit(std::make_unique<OtherData>(1));
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                // Emit a trigger test to see if the dependent tasks are blocked
                events.push_back("emitting Trigger Test");
                emit(std::make_unique<TriggerTest>());
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

TEST_CASE("Test that providers that are active are able to be triggered from other bind statements",
          "[director][triggered]") {

    // Run the module
    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting data 0",
        "emitting Trigger Test",
        "emitting simple task",
        "task 0",
        "emitting dependent task 1",
        "A: 0",
        "emitting Trigger Test",
        "A: 0",
        "emitting data 1",
        "task 1",
        "emitting dependent task 2",
        "B: 1",
        "emitting Trigger Test",
        "B: 1",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
