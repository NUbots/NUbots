#include <catch2/catch_test_macros.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

namespace {

    struct ParentTask {};
    struct Subtask1 {};
    struct Subtask2 {};
    struct Ping {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 4> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            // When the parent task runs it emits two subtasks
            on<Provide<ParentTask>>().then([this] {
                emit<Task>(std::make_unique<Subtask1>());
                emit<Task>(std::make_unique<Subtask2>());
            });

            // Subtask 1 simply records its execution
            on<Provide<Subtask1>>().then([this] { events.push_back("subtask1 executed"); });

            // When Subtask 1 is stopped it tries to re-add Subtask 2 (this used to break removal)
            on<Stop<Subtask1>>().then([this] {
                events.push_back("subtask1 stopped");
                emit<Task>(std::make_unique<Subtask2>());
            });

            // Subtask 2 only runs when Ping is emitted so we can check if it is still alive
            on<Provide<Subtask2>, Trigger<Ping>>().then([this] { events.push_back("subtask2 executed"); });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting parent task");
                emit<Task>(std::make_unique<ParentTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting ping");
                emit(std::make_unique<Ping>());
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("removing parent task");
                emit<Task>(std::unique_ptr<ParentTask>(nullptr));
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("emitting ping after removal");
                emit(std::make_unique<Ping>());
            });
        }
    };

}  // namespace

TEST_CASE("Test that removing a task removes all its subtasks even if removing one subtask tries to re-add another",
          "[director][subtask][removal][regression]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting parent task",         // Step 1
        "subtask1 executed",            // Subtask 1 runs
        "emitting ping",                // Step 2
        "subtask2 executed",            // Subtask 2 runs while still valid
        "removing parent task",         // Step 3
        "subtask1 stopped",             // Subtask 1 is stopped and attempts to re-add Subtask 2
        "emitting ping after removal",  // Step 4 – Subtask 2 should NOT run again
    };

    // If Subtask 2 remains after the parent removal this diff will make the failure obvious
    INFO(util::diff_string(expected, events));

    REQUIRE(events == expected);
}
