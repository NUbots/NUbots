#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <fmt/format.h>
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

namespace module::network::test::forwarder::bidirectional {

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
            std::string log = fmt::format("{}<{}> {} {}, Data: {}", source, scope, send ? "->" : "<-", type, data);
            events.push_back(log);
        }

        auto configure_test_forwarding(const std::string& target_name) {
            return [this, target_name] {
                auto config = std::make_unique<NetworkForwarderConfig>();
                config->targets[target_name].messages["message.network.Test"] = {true, true, false, 0.0};

                emit<Scope::INLINE>(std::move(config));
                new_event("Tester", "LOCAL", true, "ForwarderConfig", target_name);
            };
        }

        auto emit_test_msg(const std::string& message) {
            return [this, message] {
                emit(std::make_unique<Test>(message));
                new_event("Tester", "LOCAL", true, "Test", message);
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

            // Record all network "Test" messages for comparison with expected results at the end of the test
            // NOTE: This will also record "Test" messages emitted by this test reactor, so those need to be included
            //       in the expected events.
            on<MockNetwork<Test>, MainThread>().then([this](const NetworkSource& source, const Test& message) {
                new_event("Forwarder", "NETWORK", true, "Test", source.name + ", " + message.message);
            });

            // Record all locally emitted "Test" messages for comparison with expected results at the end of the test
            on<Trigger<Test>, MainThread>().then(
                [this](const Test& message) { new_event("Tester", "LOCAL", false, "Test", message.message); });

            // --- Test Steps ---

            // GIVEN the network forwarder is configured to forward the same message type to and from the same target
            on<Trigger<Step<1>>, MainThread>().then(configure_test_forwarding("Target 1"));

            // WHEN a message of the type configured to be forwarded is emitted locally
            on<Trigger<Step<2>>, MainThread>().then(emit_test_msg("Test 1"));

            // THEN the message is sent to the configured target (event logged by `on<MockNetwork<Test>>`)

            // WHEN the configured target sends a message of the same type
            on<Trigger<Step<3>>, MainThread>().then(emit_mock_network_test_msg("Target 1", "Test 1"));

            // THEN the message is emitted locally (event logged by `on<Trigger<Test>>`) and this locally emitted
            // message is NOT forwarded (no more events logged by `on<MockNetwork<Test>>`)
        }

        // The test results
        std::vector<std::string> events;
    };

}  // namespace module::network::test::forwarder::bidirectional

SCENARIO("Messages are forwarded internally and externally as configured without creating feedback loops",
         "[module][network][NetworkForwarder][NetworkForwarderBidirectional]") {

    // Create and configure a power plant
    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant plant(config);

    // Output NUClear logs as Catch2 UNSCOPED_INFO logs
    plant.install<utility::reactor::test::TestLogHandler>();

    // Install a timeout watchdog to fail the testing and shutdown the powerplant if it takes too long
    plant.install<utility::reactor::test::TestTimeout>(std::chrono::seconds(1), true);

    // Install a test reactor to run the tests. This must be installed prior to the reactor being tested
    const auto& test_reactor = plant.install<module::network::test::forwarder::bidirectional::TestReactor>();

    // Install the reactor being tested
    plant.install<module::network::NetworkForwarder>();

    // Install a test stepper to automatically trigger each `on<Step<N>>` reaction in the test reactor
    plant.install<utility::reactor::test::IdleTestStepper<3>>();

    // Start the power plant
    plant.start();

    // The events expected to occur during the test
    const std::vector<std::string> expected_events = {"Tester<LOCAL> -> ForwarderConfig, Data: Target 1",
                                                      "Tester<LOCAL> -> Test, Data: Test 1",
                                                      "Tester<LOCAL> <- Test, Data: Test 1",
                                                      "Forwarder<NETWORK> -> Test, Data: Target 1, Test 1",
                                                      "Forwarder<NETWORK> <- Test, Data: Target 1, Test 1",
                                                      "Tester<LOCAL> <- Test, Data: Test 1"};

    // Print the diff of expected events vs. actual events on test failure
    INFO(utility::strutil::diff_string(expected_events, test_reactor.events));

    // Check the actual events match the expected events
    REQUIRE(test_reactor.events == expected_events);
}
