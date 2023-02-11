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

#include <catch.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct PrimaryTask {};
    struct SecondaryTask {};
    struct TriggerTest {};

    struct SubTask {
        SubTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    struct SubSubTask {
        SubSubTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<PrimaryTask>, Trigger<TriggerTest>>().then([this](const RunInfo& info) {
                // Run SubTask when task is trigger, but not when the TriggerTest triggers the provider
                if (info.run_reason == RunInfo::RunReason::NEW_TASK) {
                    emit<Task>(std::make_unique<SubTask>("primary task"));
                }
            });

            on<Provide<SubTask>, Needs<SubSubTask>>().then(
                [this](const SubTask& t) { emit<Task>(std::make_unique<SubSubTask>(t.msg)); });

            on<Provide<SubSubTask>>().then([this](const SubSubTask& t) { events.push_back(t.msg); });

            on<Provide<SecondaryTask>, Needs<SubTask>>().then(
                [this] { emit<Task>(std::make_unique<SubTask>("secondary task")); });

            /**************
             * TEST STEPS *
             **************/
            // PrimaryTask takes ahold of SubTask and SubSubTask
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting primary task");
                emit(std::make_unique<TriggerTest>());
                emit<Task>(std::make_unique<PrimaryTask>(), 1);
            });

            // SecondaryTask needs SubTask, so it will watch
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting secondary task");
                emit<Task>(std::make_unique<SecondaryTask>(), 0);
            });

            // Remove PrimaryTask so SecondaryTask can take over
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("removing primary task");
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

TEST_CASE("Test that a watcher can take over from another provider with trigger",
          "[director][remove][watcher][trigger]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {"emitting primary task",
                                         "primary task",
                                         "emitting secondary task",
                                         "removing primary task",
                                         "secondary task"};

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
