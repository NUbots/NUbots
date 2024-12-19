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

    class TestReactor : public TestBase<TestReactor, 4> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

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
        }
    };

}  // namespace

TEST_CASE("Test that when a higher priority task is emitted it overwrites lower priority tasks",
          "[director][priority][steal]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
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
