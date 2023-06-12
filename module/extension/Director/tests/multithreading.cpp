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
#include <fmt/format.h>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct TriggerMainTask {
        TriggerMainTask(const int& loop_id_) : loop_id(loop_id_) {}
        int loop_id;
    };

    struct MainTask {
        MainTask(const int& loop_id_) : loop_id(loop_id_) {}
        int loop_id;
    };

    template <int I>
    struct SubTask {
        SubTask(const int& loop_id_) : loop_id(loop_id_), subtask_id(I) {}
        int loop_id;
        int subtask_id;
    };

    struct Dependency {
        Dependency(const int& loop_id_, const int& subtask_id_) : loop_id(loop_id_), subtask_id(subtask_id_) {}
        int loop_id;
        int subtask_id;
    };

    std::vector<std::string> events;
    const int MAX_LOOPS = 100;
    std::mutex mutex;

    void log_event(const std::string& event) {
        std::lock_guard<std::mutex> lock(mutex);
        events.push_back(event);
    }

    class TestReactor : public TestBase<TestReactor, false> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor, false>(std::move(environment)) {

            on<Provide<MainTask>>().then([this](const MainTask& t) {
                // Start of this main task loop
                log_event(fmt::format("loop {}: start", t.loop_id));

                // Emit 3 optional tasks so they can all run in parallel if possible
                log_event(fmt::format("loop {}: emitting task 1", t.loop_id));
                emit<Task>(std::make_unique<SubTask<1>>(t.loop_id), 0, true);
                log_event(fmt::format("loop {}: emitting task 2", t.loop_id));
                emit<Task>(std::make_unique<SubTask<2>>(t.loop_id), 0, true);
            });

            on<Provide<SubTask<1>>, Needs<Dependency>>().then([this](const SubTask<1>& t) {
                log_event(fmt::format("loop {}: subtask {} executed", t.loop_id, t.subtask_id));

                // Emit the dependency task
                log_event(fmt::format("loop {}: emitting dependency from subtask {}", t.loop_id, t.subtask_id));
                emit<Task>(std::make_unique<Dependency>(t.loop_id, t.subtask_id));
            });

            on<Provide<SubTask<2>>, Needs<Dependency>>().then([this](const SubTask<2>& t) {
                log_event(fmt::format("loop {}: subtask {} executed", t.loop_id, t.subtask_id));

                // Emit the dependency task
                log_event(fmt::format("loop {}: emitting dependency from subtask {}", t.loop_id, t.subtask_id));
                emit<Task>(std::make_unique<Dependency>(t.loop_id, t.subtask_id));
            });

            on<Provide<Dependency>>().then([this](const Dependency& t) {
                log_event(fmt::format("loop {}: dependent from subtask {}", t.loop_id, t.subtask_id));

                // Trigger another step of the main task loop out of band
                if (t.loop_id < MAX_LOOPS) {
                    emit(std::make_unique<TriggerMainTask>(t.loop_id + 1));
                }
                else {
                    powerplant.shutdown();
                }
            });

            on<Trigger<TriggerMainTask>>().then([this](const TriggerMainTask& t) {
                // Start of this main task loop
                log_event(fmt::format("loop {}: initiating", t.loop_id));
                emit<Task>(std::unique_ptr<MainTask>());
                emit<Task>(std::make_unique<MainTask>(t.loop_id));
            });

            on<Startup>().then([this] { emit(std::make_unique<TriggerMainTask>(0)); });
        }
    };

}  // namespace

TEST_CASE("Test that the order of tasks is stable even with multiple threads", "[director][multithread]") {

    // Run the module
    NUClear::PowerPlant::Configuration config;
    config.thread_count = 4;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {};

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
