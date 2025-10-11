#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <nuclear>
#include <string>
#include <vector>

#include "Player.hpp"

#include "message/eye/Scrubber.hpp"
#include "message/network/Test.hpp"

#include "utility/reactor/test/TestLogHandler.hpp"
#include "utility/reactor/test/TestTimeout.hpp"
#include "utility/strutil/diff_string.hpp"

namespace {

    using message::eye::ScrubberLoadRequest;
    using message::eye::ScrubberPlaybackFinished;
    using message::eye::ScrubberPlayRequest;
    using message::eye::ScrubberSetModeRequest;
    using message::eye::ScrubberState;
    using message::network::Test;

    /**
     * A test reactor that loads a file, sets the playback mode (FAST, REALTIME, or SEQUENTIAL),
     * starts playback, and records state transitions + Test messages.
     */
    class TestReactor : public NUClear::Reactor {
    public:
        struct DoTest {};

        TestReactor(std::unique_ptr<NUClear::Environment> environment, const ScrubberState::Mode& mode)
            : NUClear::Reactor(std::move(environment)) {

            on<Startup>().then([this] { emit(std::make_unique<DoTest>()); });

            on<Trigger<DoTest>, Priority::IDLE>().then([this] {
                /// Load the file
                events.push_back("Action: Load");
                auto load_request = std::make_unique<ScrubberLoadRequest>();
                load_request->files.push_back("test/valid.nbs");
                load_request->messages = {"message.network.Test"};
                emit(load_request);
            });
            on<Trigger<DoTest>, Priority::IDLE>().then([this, mode] {
                /// Set the playback mode
                events.push_back(fmt::format("Action: Set Mode {}", std::string(mode)));
                auto set_mode_request  = std::make_unique<ScrubberSetModeRequest>();
                set_mode_request->mode = mode;
                emit(set_mode_request);
            });
            on<Trigger<DoTest>, Priority::IDLE>().then([this] {
                /// Play the file
                events.push_back("Action: Play");
                emit(std::make_unique<ScrubberPlayRequest>());
            });

            on<Trigger<ScrubberState>, Priority::HIGH>().then([this](const ScrubberState& state) {
                /// Record the state
                log<INFO>(fmt::format("State: {}, Mode: {}",
                                      std::string(state.playback_state),
                                      std::string(state.playback_mode)));
                events.push_back(fmt::format("Timestamp: {}, State: {}, Mode: {}",
                                             state.timestamp.time_since_epoch().count(),
                                             std::string(state.playback_state),
                                             std::string(state.playback_mode)));
            });

            on<Trigger<Test>, Priority::HIGH>().then([this](const Test& msg) {
                /// Record the message
                log<INFO>(fmt::format("Message: {}", msg.message));
                events.push_back(fmt::format("Message: {}", msg.message));
            });

            on<Trigger<ScrubberPlaybackFinished>, Priority::HIGH>().then([this]() {
                events.push_back("Action: Finished");
                powerplant.shutdown();
            });
        }

        // Events from the player reactor
        std::vector<std::string> events;
    };

}  // namespace

SCENARIO("Player with different playback modes", "[module][nbs][Player]") {

    ScrubberState::Mode test_mode =
        GENERATE(ScrubberState::Mode::FAST, ScrubberState::Mode::REALTIME, ScrubberState::Mode::SEQUENTIAL);
    INFO("Testing playback mode: " << test_mode);

    // Create the powerplant
    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant plant(config);

    // Install ChronoController
    plant.install<NUClear::extension::ChronoController>();

    // Install the player
    plant.install<module::nbs::Player>();

    // Install the test timeout
    plant.install<utility::reactor::test::TestTimeout>(std::chrono::seconds(10));
    plant.install<utility::reactor::test::TestLogHandler>();

    // Install our combined test reactor with the chosen mode
    auto& test_reactor = plant.install<TestReactor>(test_mode);

    // Start the powerplant
    plant.start();

    // Check that the player went from ENDED -> PLAYING -> ENDED
    std::vector<std::string> expected_events = {
        "Action: Load",
        fmt::format("Timestamp: 0, State: PAUSED, Mode: REALTIME"),
        fmt::format("Action: Set Mode {}", std::string(test_mode)),
        fmt::format("Timestamp: 0, State: PAUSED, Mode: {}", std::string(test_mode)),
        fmt::format("Action: Play"),
        fmt::format("Timestamp: 0, State: PLAYING, Mode: {}", std::string(test_mode)),
        fmt::format("Message: 1"),
        fmt::format("Timestamp: 0, State: PLAYING, Mode: {}", std::string(test_mode)),
        fmt::format("Message: 2"),
        fmt::format("Timestamp: 1000000, State: PLAYING, Mode: {}", std::string(test_mode)),
        fmt::format("Message: 3"),
        fmt::format("Timestamp: 2000000, State: PLAYING, Mode: {}", std::string(test_mode)),
        fmt::format("Message: 4"),
        fmt::format("Timestamp: 3000000, State: PLAYING, Mode: {}", std::string(test_mode)),
        fmt::format("Timestamp: 3000000, State: ENDED, Mode: {}", std::string(test_mode)),
        fmt::format("Action: Finished"),
    };

    // Print the diff of expected events vs. actual events on test failure
    INFO(utility::strutil::diff_string(expected_events, test_reactor.events));

    // Check the actual events match the expected events
    REQUIRE(test_reactor.events == expected_events);
}
