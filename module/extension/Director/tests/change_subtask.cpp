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

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct ParentTask {
        ParentTask(int subtask_) : subtask(subtask_) {}
        int subtask = 0;
    };

    template <int N>
    struct SubTask {
        SubTask(const std::string& msg) : msg(msg) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 2> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            on<Provide<ParentTask>>().then([this](const ParentTask& task) {
                if (task.subtask == 1) {
                    events.push_back("parent task with subtask 1");
                    emit<Task>(std::make_unique<SubTask<1>>("from parent task"));
                }
                else {
                    events.push_back("parent task with subtask 2");
                    emit<Task>(std::make_unique<SubTask<2>>("from parent task"));
                }
            });

            on<Provide<SubTask<1>>>().then([this](const SubTask<1>& t) {  //
                events.push_back("subtask 1 " + t.msg);
                emit<Task>(std::make_unique<SubTask<2>>("from subtask 1"));
            });

            on<Provide<SubTask<2>>>().then([this](const SubTask<2>& t) {  //
                events.push_back("subtask 2 " + t.msg);
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting parent task with subtask 1");
                emit<Task>(std::make_unique<ParentTask>(1));
            });

            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting parent task with subtask 2");
                emit<Task>(std::make_unique<ParentTask>(2));
            });
        }

    private:
        ReactionHandle a_handle;
    };
}  // namespace

TEST_CASE("Test that subtasks can be changed when one subtask calls the other", "[director][subtasks][change]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting parent task with subtask 1",
        "parent task with subtask 1",
        "subtask 1 from parent task",
        "subtask 2 from subtask 1",
        "emitting parent task with subtask 2",
        "parent task with subtask 2",
        "subtask 2 from parent task",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
