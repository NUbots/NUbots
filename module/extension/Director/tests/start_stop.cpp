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
    struct Runner {
        Runner(const bool& task_) : task(task_) {}
        bool task;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Start<SimpleTask>>().then([this] { events.push_back("start"); });
            on<Provide<SimpleTask>>().then([this] { events.push_back("provide"); });
            on<Stop<SimpleTask>>().then([this] { events.push_back("stop"); });

            on<Provide<Runner>>().then([this](const Runner& runner) {
                // Emit a task to execute
                if (runner.task) {
                    events.push_back("running task");
                    emit<Task>(std::make_unique<SimpleTask>());
                }
                else {
                    events.push_back("removing task");
                }
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Emit a task to execute
                events.push_back("initiating");
                emit<Task>(std::make_unique<Runner>(true));
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emit a null task to remove the task
                events.push_back("finishing");
                emit<Task>(std::make_unique<Runner>(false));
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test that the start and stop events fire when a provider gains/loses a task", "[director][start][stop]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "initiating",
        "running task",
        "start",
        "provide",
        "finishing",
        "removing task",
        "stop",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
