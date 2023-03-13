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

    template <char id>
    struct Runner {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask>>().then([this](const SimpleTask& t) {  //
                events.push_back("simple task from " + t.msg);
            });

            on<Provide<Runner<'a'>>>().then([this] {
                events.push_back("runner a");
                emit<Task>(std::make_unique<SimpleTask>("a"));
            });

            on<Provide<Runner<'b'>>>().then([this] {
                events.push_back("runner b");
                emit<Task>(std::make_unique<SimpleTask>("b"));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("starting a");
                emit<Task>(std::make_unique<Runner<'a'>>(), 10);
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("starting b");
                emit<Task>(std::make_unique<Runner<'b'>>(), 10);
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("upping b priority");
                emit<Task>(std::make_unique<Runner<'b'>>(), 20);
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("upping a priority");
                emit<Task>(std::make_unique<Runner<'a'>>(), 20);
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

TEST_CASE("Test when two tasks of equal priority are emitted the one that was emitted first wins",
          "[director][priority][equal]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "starting a",
        "runner a",
        "simple task from a",
        "starting b",
        "runner b",
        "upping b priority",
        "runner b",
        "simple task from b",
        "upping a priority",
        "runner a",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
