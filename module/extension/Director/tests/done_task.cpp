#include <catch.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct SimpleTask {};
    struct SubTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SubTask>>().then([this] {
                events.push_back("subtask executed");
                emit<Task>(std::make_unique<Done>());
            });

            on<Provide<SimpleTask>>().then([this](const SimpleTask& task) {
                if (!executed) {
                    // Task has been executed!
                    events.push_back("task executed");
                    emit<Task>(std::make_unique<SubTask>());
                }
                else {
                    events.push_back("task reexecuted");
                }
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting task");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Startup>().then([this] {  //
                emit(std::make_unique<Step<1>>());
            });
        }

        bool executed = false;
    };
}  // namespace

TEST_CASE("Test that a done task causes the parent task that emitted it to rerun", "[director][!mayfail]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    // Check the events fired in order and only those events
    REQUIRE(events
            == std::vector<std::string>{"emitting task", "task executed", "subtask executed", "task reexecuted"});
}
