#include <catch2/catch_test_macros.hpp>
#include <fmt/format.h>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

namespace {

    // This test is testing for the case when

    // Task is emitted
    // Emit a task and a wait
    // Emit the task again
    // Emit a continue from the task
    // At this point parentage is broken


    struct CombinedTask {};
    struct SideTask {};

    /// @brief Get the current time since epoch in milliseconds
    int get_time_ms() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(NUClear::clock::now().time_since_epoch()).count();
    }

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 2> {
    public:
        int subtask_done_count = 0;

        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase(std::move(environment), false) {

            /*************************
             * Provider for CombinedTask *
             *************************/
            on<Provide<CombinedTask>>().then([this](const RunReason& run_reason) {
                switch (run_reason) {
                    case RunReason::NEW_TASK: {
                        // Emit both a Wait and a SideTask
                        events.push_back(fmt::format("combined task executed, waiting at time {}", get_time_ms()));
                        emit<Task>(std::make_unique<Wait>(NUClear::clock::now() + std::chrono::milliseconds(100)));
                        emit<Task>(std::make_unique<SideTask>());
                    } break;
                    case RunReason::SUBTASK_DONE: {
                        // The SideTask finishes first, the Wait finishes second.
                        // Only shut down once both subtasks have completed.
                        if (++subtask_done_count < 2) {
                            // Keep the other subtask (the Wait) alive while we wait for it to finish
                            emit<Task>(std::make_unique<Continue>());
                            break;
                        }
                        events.push_back(fmt::format("combined task executed, done waiting at time {}", get_time_ms()));
                        powerplant.shutdown();
                    } break;
                    default: break;
                }
            });

            /***************************
             * Provider for SideTask *
             ***************************/
            on<Provide<SideTask>>().then([this] {
                events.push_back(fmt::format("side task executed at time {}", get_time_ms()));
                // Signal that this subtask is complete so the parent gets a SUBTASK_DONE
                emit<Task>(std::make_unique<Done>());
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Freeze time at 0 so timing is deterministic
                emit(std::make_unique<NUClear::message::TimeTravel>(std::chrono::system_clock::time_point{},
                                                                    0.0,
                                                                    NUClear::message::TimeTravel::Action::RELATIVE));

                events.push_back("emitting combined task");
                emit<Task>(std::make_unique<CombinedTask>());
            });

            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Advance time to when Wait should finish (100 ms)
                emit(std::make_unique<NUClear::message::TimeTravel>(
                    NUClear::clock::now() + std::chrono::milliseconds(100),
                    0.0,
                    NUClear::message::TimeTravel::Action::ABSOLUTE));
            });
        }
    };

}  // namespace

TEST_CASE("Test that a provider emitting Wait and another task runs both", "[director][wait][parallel]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<NUClear::extension::ChronoController>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting combined task",
        "combined task executed, waiting at time 0",
        "side task executed at time 0",
        "combined task executed, done waiting at time 100",
    };

    INFO(util::diff_string(expected, events));

    REQUIRE(events == expected);
}
