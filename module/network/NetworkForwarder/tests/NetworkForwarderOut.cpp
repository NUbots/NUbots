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

namespace module::network::test::forwarder::out {

    using message::network::MessageRequest;
    using message::network::NetworkForwarderConfig;
    using message::network::Test;
    using NUClear::message::NetworkJoin;
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
                config->targets[target_name].messages["message.network.Test"] = {true, false, false, 0.0};

                emit<Scope::INLINE>(std::move(config));
                new_event("Tester", "LOCAL", true, "ForwarderConfig", target_name);
            };
        }

        auto configure_no_forwarding() {
            return [this] {
                auto config = std::make_unique<NetworkForwarderConfig>();
                emit<Scope::INLINE>(std::move(config));
                new_event("Tester", "LOCAL", true, "ForwarderConfig", "None");
            };
        }

        auto emit_test_msg(const std::string& message) {
            return [this, message] {
                emit(std::make_unique<Test>(message));
                new_event("Tester", "LOCAL", true, "Test", message);
            };
        }

        auto emit_network_join_msg(const std::string& source_name) {
            return [this, source_name] {
                auto msg  = std::make_unique<NetworkJoin>();
                msg->name = source_name;
                emit(msg);
                new_event("Tester", "LOCAL", true, "NetworkJoin", source_name);
            };
        }

        auto emit_mock_test_msg_request(const std::string& source_name) {
            return [this, source_name] {
                auto msg  = std::make_unique<MessageRequest>();
                msg->name = "message.network.Test";
                auto src  = NetworkSource{};
                src.name  = source_name;
                emit<Scope::MOCK_NETWORK>(msg, src);
                new_event("Forwarder", "NETWORK", false, "MessageRequest", source_name);
            };
        }

    public:
        TestReactor(std::unique_ptr<NUClear::Environment> environment) : MockNetworkReactor(std::move(environment)) {

            // --- Test Setup ---

            // Record all network "Test" messages for comparison with expected results at the end of the test
            on<MockNetwork<Test>, MainThread>().then([this](const NetworkSource& source, const Test& message) {
                new_event("Forwarder", "NETWORK", true, "Test", source.name + ", " + message.message);
            });

            // --- Test Steps ---

            // GIVEN the network forwarder is configured to forward one message type to one target
            on<Trigger<Step<1>>, MainThread>().then(configure_test_forwarding("Target 1"));

            // WHEN a message of the type configured to be forwarded is emitted locally
            on<Trigger<Step<2>>, MainThread>().then(emit_test_msg("Test 1"));

            // THEN the message is sent to the configured target (event logged by `on<MockNetwork<Test>>`)

            // AND WHEN the configured target joins the network
            on<Trigger<Step<3>>, MainThread>().then(emit_network_join_msg("Target 1"));

            // THEN the message is sent to the configured target again (event logged by `on<MockNetwork<Test>>`)

            // AND WHEN the configured target requests the previously forwarded messages
            on<Trigger<Step<4>>, MainThread>().then(emit_mock_test_msg_request("Target 1"));

            // THEN the message is sent to the configured target again (event logged by `on<MockNetwork<Test>>`)

            // AND WHEN the network forwarder is reconfigured to not forward any message types
            on<Trigger<Step<5>>, MainThread>().then(configure_no_forwarding());

            // AND a message of the type previously configured to be forwarded is emitted locally
            on<Trigger<Step<6>>, MainThread>().then(emit_test_msg("Test 1"));

            // THEN no messages are sent to the configured target

            // AND WHEN the previously configured target joins the network
            on<Trigger<Step<7>>, MainThread>().then(emit_network_join_msg("Target 1"));

            // THEN no messages are sent to the configured target

            // AND WHEN the previously configured target requests the previously forwarded messages
            on<Trigger<Step<8>>, MainThread>().then(emit_mock_test_msg_request("Target 1"));

            // THEN no messages are sent to the configured target
        }

        // The test results
        std::vector<std::string> events;
    };

}  // namespace module::network::test::forwarder::out

SCENARIO("Messages are forwarded externally as configured",
         "[module][network][NetworkForwarder][NetworkForwarderOut]") {

    // Create and configure a power plant
    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant plant(config);

    // Output NUClear logs as Catch2 UNSCOPED_INFO logs
    plant.install<utility::reactor::test::TestLogHandler>();

    // Install a timeout watchdog to fail the testing and shutdown the powerplant if it takes too long
    plant.install<utility::reactor::test::TestTimeout>(std::chrono::seconds(1), true);

    // Install a test reactor to run the tests. This must be installed prior to the reactor being tested
    const auto& test_reactor = plant.install<module::network::test::forwarder::out::TestReactor>();

    // Install the reactor being tested
    plant.install<module::network::NetworkForwarder>();

    // Install a test stepper to automatically trigger each `on<Step<N>>` reaction in the test reactor
    plant.install<utility::reactor::test::IdleTestStepper<8>>();

    // Start the power plant
    plant.start();

    // The events expected to occur during the test
    const std::vector<std::string> expected_events = {"Tester<LOCAL> -> ForwarderConfig, Data: Target 1",
                                                      "Tester<LOCAL> -> Test, Data: Test 1",
                                                      "Forwarder<NETWORK> -> Test, Data: Target 1, Test 1",
                                                      "Tester<LOCAL> -> NetworkJoin, Data: Target 1",
                                                      "Forwarder<NETWORK> -> Test, Data: Target 1, Test 1",
                                                      "Forwarder<NETWORK> <- MessageRequest, Data: Target 1",
                                                      "Forwarder<NETWORK> -> Test, Data: Target 1, Test 1",
                                                      "Tester<LOCAL> -> ForwarderConfig, Data: None",
                                                      "Tester<LOCAL> -> Test, Data: Test 1",
                                                      "Tester<LOCAL> -> NetworkJoin, Data: Target 1",
                                                      "Forwarder<NETWORK> <- MessageRequest, Data: Target 1"};

    // Print the diff of expected events vs. actual events on test failure
    INFO(utility::strutil::diff_string(expected_events, test_reactor.events));

    // Check the actual events match the expected events
    REQUIRE(test_reactor.events == expected_events);
}
