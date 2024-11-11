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

    template <int i>
    struct SimpleTask {
        SimpleTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };
    template <int i>
    struct ComplexTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 2> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            // First complex task needs both simple tasks
            on<Provide<ComplexTask<1>>, Needs<SimpleTask<1>>, Needs<SimpleTask<2>>>().then(
                [this](const ComplexTask<1>& t) {
                    emit<Task>(std::make_unique<SimpleTask<1>>(t));
                    emit<Task>(std::make_unique<SimpleTask<2>>(t));
                });
            // Second complex task only needs one
            on<Provide<ComplexTask<2>>, Needs<SimpleTask<1>>>().then(
                [this](const ComplexTask<2>& t) { emit<Task>(std::make_unique<SimpleTask<1>>(t)); });

            // Monitor which events run and by who
            on<Provide<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) { finish(t, "Provide"); });
            on<Provide<SimpleTask<2>>>().then([this](const SimpleTask<2>& t) { finish(t, "Provide"); });
            on<Start<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) { finish(t, "Start"); });
            on<Start<SimpleTask<2>>>().then([this](const SimpleTask<2>& t) { finish(t, "Start"); });
            on<Stop<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) { finish(t, "Stop"); });
            on<Stop<SimpleTask<2>>>().then([this](const SimpleTask<2>& t) { finish(t, "Stop"); });
            on<Start<ComplexTask<1>>>().then([this](const ComplexTask<1>& t) { finish(t, "Start"); });
            on<Start<ComplexTask<2>>>().then([this](const ComplexTask<2>& t) { finish(t, "Start"); });
            on<Stop<ComplexTask<1>>>().then([this](const ComplexTask<1>& t) { finish(t, "Stop"); });
            on<Stop<ComplexTask<2>>>().then([this](const ComplexTask<2>& t) { finish(t, "Stop"); });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>>().then([this] { emit<Task>(std::make_unique<ComplexTask<1>>(), 50); });
            on<Trigger<Step<2>>>().then([this] { emit<Task>(std::make_unique<ComplexTask<2>>(), 100); });
        }
    };

}  // namespace

TEST_CASE("Tests that if a provider loses one of its dependent needs it stops running everything",
          "[director][priority][needs]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "requesting complex task 1",
        "start complex 1",
        "emitting complex task 1",
        "start 1: complex 1",
        "start 2: complex 1",
        "p1: complex 1",
        "p2: complex 1",
        "requesting complex task 2",
        "start complex 2",
        "emitting complex task 2",
        "stop complex 1",
        "stop 2: complex 1",
        "p1: complex 2",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
