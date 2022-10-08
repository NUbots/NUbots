#include <catch.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct SimpleTask {};

    template <int a, int b>
    struct SubTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask>>().then([this] {
                // Task has been executed!
                events.push_back("simple task executed");
                emit<Task>(std::make_unique<SubTask<1, 0>>());
                emit<Task>(std::make_unique<SubTask<2, 0>>());
                emit<Task>(std::make_unique<SubTask<3, 0>>());
            });

            on<Provide<SubTask<1, 0>>>().then([this] {
                events.push_back("subtask 1,0 executed");
                emit<Task>(std::make_unique<SubTask<1, 1>>());
                emit<Task>(std::make_unique<SubTask<1, 2>>());
            });
            on<Provide<SubTask<1, 1>>>().then([this] { events.push_back("subtask 1,1 executed"); });
            on<Provide<SubTask<1, 2>>>().then([this] { events.push_back("subtask 1,2 executed"); });

            on<Provide<SubTask<2, 0>>>().then([this] {
                events.push_back("subtask 2,0 executed");
                emit<Task>(std::make_unique<SubTask<2, 1>>());
                emit<Task>(std::make_unique<SubTask<2, 2>>());
            });
            on<Provide<SubTask<2, 1>>>().then([this] { events.push_back("subtask 2,1 executed"); });
            on<Provide<SubTask<2, 2>>>().then([this] { events.push_back("subtask 2,2 executed"); });

            on<Provide<SubTask<3, 0>>>().then([this] {
                events.push_back("subtask 3,0 executed");
                emit<Task>(std::make_unique<SubTask<3, 1>>());
                emit<Task>(std::make_unique<SubTask<3, 2>>());
            });
            on<Provide<SubTask<3, 1>>>().then([this] { events.push_back("subtask 3,1 executed"); });
            on<Provide<SubTask<3, 2>>>().then([this] { events.push_back("subtask 3,2 executed"); });

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
    };
}  // namespace

TEST_CASE("Test that a tree of simple tasks are executed through the director", "[director][simple]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting task",
        "simple task executed",
        "subtask 1,0 executed",
        "subtask 2,0 executed",
        "subtask 3,0 executed",
        "subtask 1,1 executed",
        "subtask 1,2 executed",
        "subtask 2,1 executed",
        "subtask 2,2 executed",
        "subtask 3,1 executed",
        "subtask 3,2 executed",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
