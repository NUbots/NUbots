#include <catch.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct SimpleTask {
        SimpleTask(std::string msg) : msg(msg) {}
        std::string msg;
    };

    template <int i>
    struct UniqueProvider {
        UniqueProvider(std::string msg) : msg(msg) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

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
                // Emit a third task with middling priority that should't displace the high priority task
                events.push_back("Starting very high priority provider");
                emit<Task>(std::make_unique<UniqueProvider<4>>("very high"), 200);
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

TEST_CASE("Test that when a higher priority task is emitted it overwrites lower priority tasks",
          "[director][!mayfail]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    // Check the events fired in order and only those events
    REQUIRE(events
            == std::vector<std::string>{
                "Starting low priority provider",
                "trying: low",
                "low"
                "Starting high priority provider",
                "trying: high",
                "high"
                "Starting middling priority provider",
                "trying: middling",
                "Starting very high priority provider",
                "trying: very high",
                "very high",
            });
}
