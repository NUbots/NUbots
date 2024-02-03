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

    NUClear::Configuration config;
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
