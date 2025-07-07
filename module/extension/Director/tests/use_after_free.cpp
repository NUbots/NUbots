/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

    struct TaskA {};
    struct TaskB {};
    struct TaskC {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 2> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            // TaskA provider that emits multiple subtasks
            on<Provide<TaskA>>().then([this] {
                events.push_back("TaskA executed - emitting subtasks");

                // Emit multiple subtasks - this creates entries in the solutions vector
                emit<Task>(std::make_unique<TaskB>(), 10);
                emit<Task>(std::make_unique<TaskC>(), 20);

                // Immediately emit a new TaskA with different priority
                // This should cause the current TaskA to be marked as dying and removed
                // while the solutions vector still holds references to TaskB and TaskC
                emit<Task>(std::make_unique<TaskA>(), 1000);
            });

            // TaskB that gets created but may be removed before solutions vector is destroyed
            on<Provide<TaskB>>().then([this] {
                events.push_back("TaskB executed");

                // Force re-entrant execution that might cause task removal
                // Emit nullptr to trigger task cleanup
                emit<Task>(std::unique_ptr<TaskB>(nullptr));
                emit<Task>(std::unique_ptr<TaskC>(nullptr));
            });

            // TaskC that gets created but may be removed before solutions vector is destroyed
            on<Provide<TaskC>>().then([this] { events.push_back("TaskC executed"); });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>>().then([this] {
                events.push_back("Step 1: Starting cascade");
                // Start the cascade that should create the use-after-free scenario
                emit<Task>(std::make_unique<TaskA>(), 50);
            });

            on<Trigger<Step<2>>>().then([this] {
                events.push_back("Step 2: Force cleanup");
                // Force additional cleanup
                emit<Task>(std::unique_ptr<TaskA>(nullptr));
            });
        }
    };
}  // namespace

TEST_CASE("Regression test for solutions vector use-after-free fix in run_tasks",
          "[director][use-after-free][regression]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    // This test serves as a regression test for a use-after-free bug that occurred in run_tasks.
    //
    // Original Issue:
    // - run_tasks() created a std::vector<Solution> solutions containing complex objects
    // - These Solution objects held references to DirectorTask shared_ptrs
    // - During execution, re-entrant calls could cause tasks to be removed via remove_task()
    // - When run_tasks() returned, the solutions vector destructor would try to destroy
    //   shared_ptr<DirectorTask> objects that had already been freed, causing a segfault
    //
    // The Fix:
    // - Wrapped the solutions handling logic in a lambda function that executes immediately
    // - This ensures all solution-related objects are destroyed before any potential
    //   re-entrant task removal can occur
    // - The lambda creates a clear scope boundary where solutions must be cleaned up
    //
    // This test exercises complex task emission patterns that could potentially trigger
    // the original bug scenario, but with the fix in place, should complete without crashing.

    REQUIRE(!events.empty());

    // Verify we executed the cascade of tasks
    bool found_task_a = false;
    for (const auto& event : events) {
        if (event.find("TaskA executed") != std::string::npos) {
            found_task_a = true;
            break;
        }
    }
    REQUIRE(found_task_a);

    INFO("Events that occurred:");
    for (const auto& event : events) {
        INFO("  - " + event);
    }

    // The primary success criterion is completing without a segmentation fault
    // If the original bug were present, this test would likely crash during
    // the destruction of solution vectors containing freed task references
    SUCCEED("Completed without segfault - solutions vector lifecycle fix is working");
}
