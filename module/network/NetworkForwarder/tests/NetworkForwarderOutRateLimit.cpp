#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <fmt/format.h>
#include <nuclear>
#include <thread>

#include "NetworkForwarder.hpp"

#include "message/network/MessageRequest.hpp"
#include "message/network/NetworkForwarderConfig.hpp"
#include "message/network/Test.hpp"

#include "utility/reactor/test/IdleTestStepper.hpp"
#include "utility/reactor/test/MockNetworkReactor.hpp"
#include "utility/reactor/test/TestLogHandler.hpp"
#include "utility/reactor/test/TestTimeout.hpp"
#include "utility/strutil/diff_string.hpp"

namespace module::network::test::forwarder::out::ratelimit {

    using message::network::NetworkForwarderConfig;
    using message::network::Test;
    using utility::reactor::test::Step;

    class TestReactor : public utility::reactor::test::MockNetworkReactor<TestReactor> {
    private:
        void new_event(const std::string& source,
                       const std::string& scope,
                       const bool& send,
                       const std::string& type,
                       const std::string& data) {
            std::string log = fmt::format("{}<{}> {} {}: {}", source, scope, send ? "->" : "<-", type, data);
            events.push_back(log);
        }

        auto configure_test_forwarding(const std::string& target_name, const int& msgs_per_s) {
            return [this, target_name, msgs_per_s] {
                auto config = std::make_unique<NetworkForwarderConfig>();
                config->targets[target_name].messages["message.network.Test"] = {true,
                                                                                 false,
                                                                                 false,
                                                                                 static_cast<double>(msgs_per_s)};

                emit<Scope::INLINE>(std::move(config));

                std::string log =
                    fmt::format("{}, limit: {}/s", target_name, msgs_per_s == 0 ? "inf" : std::to_string(msgs_per_s));
                new_event("Tester", "LOCAL", true, "Config", log);
            };
        }

        auto emit_test_msg(const std::string& message) {
            return [this, message] {
                emit(std::make_unique<Test>(message));
                new_event("Tester", "LOCAL", true, "Test", message);
            };
        }

        auto advance_time_and_stop(const int& duration_ms) {
            return [this, duration_ms] {
                emit<Scope::INLINE>(std::make_unique<NUClear::message::TimeTravel>(
                    NUClear::clock::now() + std::chrono::milliseconds(duration_ms),
                    0.0,
                    NUClear::message::TimeTravel::Action::ABSOLUTE));
            };
        }

    public:
        TestReactor(std::unique_ptr<NUClear::Environment> environment) : MockNetworkReactor(std::move(environment)) {

            // Go back to 1970 when testing was easier
            emit<Scope::INLINE>(
                std::make_unique<NUClear::message::TimeTravel>(NUClear::clock::time_point(NUClear::clock::duration(0)),
                                                               0.0,
                                                               NUClear::message::TimeTravel::Action::ABSOLUTE));

            // --- Test Setup ---

            // Record all network "Test" messages for comparison with expected results at the end of the test
            on<MockNetwork<Test>, MainThread>().then([this](const NetworkSource& source, const Test& message) {
                new_event("Forwarder", "NETWORK", true, "Test", fmt::format("{}, {}", source.name, message.message));
            });

            // --- Test Steps ---

            // Freeze time so that we can control the passage of time for this time-sensitive test
            on<Trigger<Step<1>>, MainThread>().then(advance_time_and_stop(0));

            // GIVEN the network forwarder is configured with a maximum rate of 10 messages/s (1 every 100ms)
            on<Trigger<Step<2>>, MainThread>().then(configure_test_forwarding("Target 1", 10));

            // WHEN a message of the configured type is emitted locally at a rate quicker than the rate limit
            on<Trigger<Step<3>>, MainThread>().then(emit_test_msg("Test 1"));  // Should pass
            on<Trigger<Step<4>>, MainThread>().then(advance_time_and_stop(50));
            on<Trigger<Step<5>>, MainThread>().then(emit_test_msg("Test 2"));  // Should be blocked

            // THEN only the first message is sent to the configured target (see `on<MockNetwork<Test>>`)

            // AND WHEN the same message type is emitted locally after the rate limit period has elapsed
            on<Trigger<Step<6>>, MainThread>().then(advance_time_and_stop(100));
            on<Trigger<Step<7>>, MainThread>().then(emit_test_msg("Test 3"));  // Should pass

            // THEN the message is sent to the configured target (see `on<MockNetwork<Test>>`)

            // AND WHEN the network forwarder is reconfigured to remove rate limiting
            on<Trigger<Step<8>>, MainThread>().then(configure_test_forwarding("Target 1", 0));

            // AND the same message type is emitted locally quicker than the original rate limit
            on<Trigger<Step<9>>, MainThread>().then(emit_test_msg("Test 4"));
            on<Trigger<Step<10>>, MainThread>().then(advance_time_and_stop(50));
            on<Trigger<Step<11>>, MainThread>().then(emit_test_msg("Test 5"));

            // THEN both messages are sent to the configured target (see `on<MockNetwork<Test>>`)
        }

        // The test results
        std::vector<std::string> events;
    };

}  // namespace module::network::test::forwarder::out::ratelimit

SCENARIO("Messages are forwarded externally with a rate limit applied as configured",
         "[module][network][NetworkForwarder][NetworkForwarderOutRateLimit]") {

    // Create and configure a power plant
    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant plant(config);

    // Output NUClear logs as Catch2 UNSCOPED_INFO logs
    plant.install<utility::reactor::test::TestLogHandler>();

    // Install a timeout watchdog to fail the testing and shutdown the powerplant if it takes too long
    plant.install<utility::reactor::test::TestTimeout>(std::chrono::seconds(1), true);

    // Install a chrono controller to manipulate time
    plant.install<NUClear::extension::ChronoController>();

    // Install a test reactor to run the tests. This must be installed prior to the reactor being tested
    const auto& test_reactor = plant.install<module::network::test::forwarder::out::ratelimit::TestReactor>();

    // Install the reactor being tested
    plant.install<module::network::NetworkForwarder>();

    // Install a test stepper to automatically trigger each `on<Step<N>>` reaction in the test reactor
    plant.install<utility::reactor::test::IdleTestStepper<11>>();

    // Start the power plant
    plant.start();

    // The events expected to occur during the test
    const std::vector<std::string> expected_events = {"Tester<LOCAL> -> Config: Target 1, limit: 10/s",
                                                      "Tester<LOCAL> -> Test: Test 1",
                                                      "Forwarder<NETWORK> -> Test: Target 1, Test 1",
                                                      "Tester<LOCAL> -> Test: Test 2",
                                                      "Tester<LOCAL> -> Test: Test 3",
                                                      "Forwarder<NETWORK> -> Test: Target 1, Test 3",
                                                      "Tester<LOCAL> -> Config: Target 1, limit: inf/s",
                                                      "Tester<LOCAL> -> Test: Test 4",
                                                      "Forwarder<NETWORK> -> Test: Target 1, Test 4",
                                                      "Tester<LOCAL> -> Test: Test 5",
                                                      "Forwarder<NETWORK> -> Test: Target 1, Test 5"};

    // Print the diff of expected events vs. actual events on test failure
    INFO(utility::strutil::diff_string(expected_events, test_reactor.events));

    // Check the actual events match the expected events
    REQUIRE(test_reactor.events == expected_events);
}
