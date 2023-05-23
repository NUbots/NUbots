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

    struct SimpleTask {
        SimpleTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    template <int i>
    struct UniqueProvider {
        UniqueProvider(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            // Store the tasks we are given to run
            on<Provide<SimpleTask>>().then([this](const SimpleTask& t) { events.push_back(t.msg); });

            // Unique providers to be siblings of each other in terms of priority
            on<Provide<UniqueProvider<1>>>().then([this](const UniqueProvider<1>& t) {  //
                events.push_back("trying: " + t.msg);
                emit<Task>(std::make_unique<SimpleTask>(t.msg));
            });
            on<Provide<UniqueProvider<2>>>().then([this](const UniqueProvider<2>& t) {  //
                events.push_back("trying: " + t.msg);
                emit<Task>(std::make_unique<SimpleTask>(t.msg));
            });
            on<Provide<UniqueProvider<3>>>().then([this](const UniqueProvider<3>& t) {  //
                events.push_back("trying: " + t.msg);
                emit<Task>(std::make_unique<SimpleTask>(t.msg));
            });
            on<Provide<UniqueProvider<4>>>().then([this](const UniqueProvider<4>& t) {  //
                events.push_back("trying: " + t.msg);
                emit<Task>(std::make_unique<SimpleTask>(t.msg));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Emit a task with low priority
                events.push_back("Starting low priority provider");
                emit<Task>(std::make_unique<UniqueProvider<1>>("low"), 1);
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emit a task with high priority
                events.push_back("Starting high priority provider");
                emit<Task>(std::make_unique<UniqueProvider<2>>("high"), 100);
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                // Emit a third task with middling priority that should't displace the high priority task
                events.push_back("Starting middling priority provider");
                emit<Task>(std::make_unique<UniqueProvider<3>>("middling"), 50);
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                // Emit a fourth task with very high priority that should't displace the high priority task
                events.push_back("Starting very high priority provider");
                emit<Task>(std::make_unique<UniqueProvider<4>>("very high"), 200);
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

TEST_CASE("Test that when a higher priority task is emitted it overwrites lower priority tasks",
          "[director][priority][steal]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "Starting low priority provider",
        "trying: low",
        "low",
        "Starting high priority provider",
        "trying: high",
        "high",
        "Starting middling priority provider",
        "trying: middling",
        "Starting very high priority provider",
        "trying: very high",
        "very high",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
