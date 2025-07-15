#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <fmt/format.h>
#include <iostream>
#include <nuclear>

#include "NetworkForwarder.hpp"

#include "message/network/MessageRequest.hpp"
#include "message/network/NetworkForwarderConfig.hpp"
#include "message/network/Test.hpp"

#include "utility/reactor/test/IdleTestStepper.hpp"
#include "utility/reactor/test/MockNetworkReactor.hpp"
#include "utility/reactor/test/TestLogHandler.hpp"
#include "utility/reactor/test/TestTimeout.hpp"
#include "utility/strutil/diff_string.hpp"

namespace module::network::test::forwarder::in::disable {

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
            std::string log = fmt::format("{}<{}> {} {}, {}", source, scope, send ? "->" : "<-", type, data);
            events.push_back(log);
        }

        auto configure_test_forwarding(const std::string& target_name) {
            return [this, target_name] {
                auto config = std::make_unique<NetworkForwarderConfig>();
                config->targets[target_name].messages["message.network.Test"] = {false, true, false, 0.0};

                emit<Scope::INLINE>(std::move(config));
                new_event("Tester", "LOCAL", true, "Config", target_name + ", 'Test' enabled");
            };
        }

        auto disable_test_forwarding(const std::string& target_name) {
            return [this, target_name] {
                auto config = std::make_unique<NetworkForwarderConfig>();
                config->targets[target_name].messages["message.network.Test"] = {false, false, false, 0.0};

                emit<Scope::INLINE>(std::move(config));
                new_event("Tester", "LOCAL", true, "Config", target_name + ", 'Test' disabled");
            };
        }

        auto emit_mock_network_test_msg(const std::string& source_name, const std::string& message) {
            return [this, source_name, message] {
                auto msg     = std::make_unique<Test>();
                msg->message = message;
                auto src     = NetworkSource{};
                src.name     = source_name;
                emit<Scope::MOCK_NETWORK>(msg, src);
                new_event("Forwarder", "NETWORK", false, "Test", source_name + ", " + message);
            };
        }

    public:
        TestReactor(std::unique_ptr<NUClear::Environment> environment) : MockNetworkReactor(std::move(environment)) {

            // --- Test Setup ---

            // Record all locally emitted "Test" messages for comparison with expected results at the end of the test
            on<Trigger<Test>, MainThread>().then(
                [this](const Test& message) { new_event("Tester", "LOCAL", false, "Test", message.message); });

            // --- Test Steps ---

            // GIVEN the network forwarder is configured to forward one message type from one target
            on<Trigger<Step<1>>, MainThread>().then(configure_test_forwarding("Target 1"));

            // WHEN the configured target sends a message of the type configured to be forwarded
            on<Trigger<Step<2>>, MainThread>().then(emit_mock_network_test_msg("Target 1", "Test 1"));

            // THEN the message is emitted locally (event logged by `on<Trigger<Test>>`)

            // AND WHEN the network forwarder is reconfigured to disable forwarding for the same message type
            on<Trigger<Step<3>>, MainThread>().then(disable_test_forwarding("Target 1"));

            // AND the previously configured target sends a message of the type previously configured to be forwarded
            on<Trigger<Step<4>>, MainThread>().then(emit_mock_network_test_msg("Target 1", "Test 1"));

            // THEN no messages are emitted locally
        }

        // The test results
        std::vector<std::string> events;
    };

}  // namespace module::network::test::forwarder::in::disable

SCENARIO("Inbound message forwarding is disabled as configured",
         "[module][network][NetworkForwarder][NetworkForwarderInDisable]") {

    // Create and configure a power plant
    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant plant(config);

    // Output NUClear logs as Catch2 UNSCOPED_INFO logs
    plant.install<utility::reactor::test::TestLogHandler>();

    // Install a timeout watchdog to fail the testing and shutdown the powerplant if it takes too long
    plant.install<utility::reactor::test::TestTimeout>(std::chrono::seconds(1), true);

    // Install a test reactor to run the tests. This must be installed prior to the reactor being tested
    const auto& test_reactor = plant.install<module::network::test::forwarder::in::disable::TestReactor>();

    // Install the reactor being tested
    plant.install<module::network::NetworkForwarder>();

    // Install a test stepper to automatically trigger each `on<Step<N>>` reaction in the test reactor
    plant.install<utility::reactor::test::IdleTestStepper<4>>();

    // Start the power plant
    plant.start();

    // The events expected to occur during the test
    const std::vector<std::string> expected_events = {"Tester<LOCAL> -> Config, Target 1, 'Test' enabled",
                                                      "Forwarder<NETWORK> <- Test, Target 1, Test 1",
                                                      "Tester<LOCAL> <- Test, Test 1",
                                                      "Tester<LOCAL> -> Config, Target 1, 'Test' disabled",
                                                      "Forwarder<NETWORK> <- Test, Target 1, Test 1"};

    // Print the diff of expected events vs. actual events on test failure
    INFO(utility::strutil::diff_string(expected_events, test_reactor.events));

    // Check the actual events match the expected events
    REQUIRE(test_reactor.events == expected_events);
}
