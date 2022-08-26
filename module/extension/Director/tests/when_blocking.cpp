#include <catch.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct SimpleTask {};

    struct Condition {
        enum Value { BLOCK, ALLOW } value;
        Condition(const Value& v) : value(v) {}
        operator int() const {
            return value;
        }
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask>, When<Condition, std::equal_to, Condition::ALLOW>>().then([this] {
                // Task has been executed!
                events.push_back("task executed");
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // At this point condition hasn't been emitted, so this task should be blocked by default
                events.push_back("emitting task");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emitting a blocked condition
                events.push_back("emitting blocked condition");
                emit(std::make_unique<Condition>(Condition::BLOCK));
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                // Emitting the task again to ensure it's not executed
                events.push_back("emitting task again");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                // This should make it run since it's now allowed
                events.push_back("emitting allowed condition");
                emit(std::make_unique<Condition>(Condition::ALLOW));
            });
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                // This task should run fine since it's allowed
                events.push_back("emitting task again #2");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
                emit(std::make_unique<Step<5>>());
            });
        }
    };
}  // namespace

TEST_CASE("Test that the when keyword blocks and allows running as expected", "[director][when][blocking][!mayfail]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting task",
        "emitting blocked condition",
        "emitting task again",
        "emitting allowed condition",
        "task executed",
        "emitting task again #2",
        "task executed",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
