// Test that if a provider is disabled, it is ignored from selection
#include <catch.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct SimpleTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            a_handle = on<Provide<SimpleTask>>().then([this] {  //
                events.push_back("version a");
            });

            on<Provide<SimpleTask>>().then([this] {  //
                events.push_back("version b");
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting task 1");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("disabling a");
                a_handle.disable();
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting task 2");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("enabling a");
                a_handle.enable();
            });
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                events.push_back("emitting task 3");
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

    private:
        ReactionHandle a_handle;
    };
}  // namespace

TEST_CASE("Test that disabled providers are not considered when choosing a provider", "[director][simple]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting task 1",
        "version a",
        "disabling a",
        "emitting task 2",
        "version b",
        "enabling a",
        "emitting task 3",
        "version a",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
