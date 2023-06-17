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

    struct SimpleTask {};
    struct SubTask {
        SubTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };
    struct Finished {
        Finished(const bool& f_) : f(f_) {}
        bool f;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SubTask>, Trigger<Finished>>().then([this](const SubTask& t, const Finished& f) {
                events.push_back("subtask executed by " + t.msg);
                if (f.f) {
                    events.push_back("finished with " + t.msg);
                    emit<Task>(std::make_unique<Done>());
                }
            });

            on<Provide<SimpleTask>>().then([this] {
                if (!executed) {
                    // Task has been executed!
                    executed = true;
                    events.push_back("simple task executed");
                    emit<Task>(std::make_unique<SubTask>("simple task"));
                }
                else {
                    events.push_back("simple task reexecuted");
                    emit<Task>(std::make_unique<Done>());
                }
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting finished with false");
                emit(std::make_unique<Finished>(false));
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting root subtask");
                emit<Task>(std::make_unique<SubTask>("root"), 20);
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting simple task");
                emit<Task>(std::make_unique<SimpleTask>(), 10);
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("emitting finished with true");
                emit(std::make_unique<Finished>(true));
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
            });
        }

        bool executed = false;
    };
}  // namespace

TEST_CASE("Test that a done task removes a root task if it was the next in the chain", "[director][done]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting finished with false",
        "emitting root subtask",
        "subtask executed by root",
        "emitting simple task",
        "simple task executed",
        "emitting finished with true",
        "subtask executed by root",
        "finished with root",
        "subtask executed by simple task",
        "finished with simple task",
        "simple task reexecuted",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
