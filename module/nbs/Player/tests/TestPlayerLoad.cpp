#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <fmt/format.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <nuclear>
#include <string>
#include <vector>

#include "Player.hpp"

#include "message/eye/Scrubber.hpp"

#include "utility/reactor/test/IdleTestStepper.hpp"
#include "utility/reactor/test/TestTimeout.hpp"

namespace {

    using message::eye::ScrubberLoadRequest;

    class TestReactor : public NUClear::Reactor {
    public:
        template <int i>
        using Step = utility::reactor::test::Step<i>;

        TestReactor(std::unique_ptr<NUClear::Environment> environment) : NUClear::Reactor(std::move(environment)) {

            on<Trigger<Step<1>>>().then([this] {
                // Load invalid file (non-existent)
                auto load_request = std::make_unique<ScrubberLoadRequest>();
                load_request->files.push_back("test/non-existent.nbs");
                emit<Scope::INLINE>(load_request);
            });

            on<Trigger<Step<2>>>().then([this] {
                // Load with no files provided
                auto load_request = std::make_unique<ScrubberLoadRequest>();
                emit<Scope::INLINE>(load_request);
            });

            on<Trigger<Step<3>>>().then([this] {
                // Load valid file
                auto load_request = std::make_unique<ScrubberLoadRequest>();
                load_request->files.push_back("test/valid.nbs");
                emit<Scope::INLINE>(load_request);
            });

            on<Trigger<ScrubberLoadRequest::Response>>().then([this](const ScrubberLoadRequest::Response& response) {
                // Store the result of the load request
                events.push_back(response.rpc.ok);
            });
        }

        /// Events from the player reactor
        std::vector<bool> events;
    };


}  // namespace


SCENARIO("Player load", "[module][nbs][Player][load]") {
    // Create the powerplant
    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant plant(config);

    // Install ChronoController
    plant.install<NUClear::extension::ChronoController>();

    // Install player
    plant.install<module::nbs::Player>();

    plant.install<utility::reactor::test::IdleTestStepper<3>>();
    plant.install<utility::reactor::test::TestTimeout>();

    // Install test reactor
    auto& reactor = plant.install<TestReactor>();

    // Start the powerplant
    plant.start();

    // Check if the events are as expected
    std::vector<bool> expected_events = {false, false, true};
    REQUIRE(reactor.events == expected_events);
}
