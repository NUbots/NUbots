#include "TestModule.hpp"

#include "message/PingPong.hpp"

#include "extension/Configuration.hpp"

namespace module {

using message::Ping;
using message::Pong;

using extension::Configuration;

TestModule::TestModule(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("TestModule.yaml").then([this](const Configuration& config) {

        // Use configuration here from file TestModule.yaml
        // this->log_level = config["log_level"].as<NUClear::LogLevel>();
        log<INFO>("Increment is of size", cfg.increment);
    });

    on<Every<2, std::chrono::seconds>, With<Ping>>().then([this](const Ping& ping) {
        // Print the Ping message!
        log<INFO>("Ping count", ping.count);

        // Make a Pong message to send
        auto pong = std::make_unique<Pong>();
        pong->count = ping.count + cfg.increment;

        // Send the message
        emit(pong);
    });

    on<Trigger<Pong>>().then([this](const Pong& pong) {
       // Print the Pong message!
        log<INFO>("Pong count", pong.count);

        // Make a Ping message to send
        auto ping = std::make_unique<Ping>();
        ping->count = pong.count + cfg.increment;

        // Send the message
        emit(ping);
    });

    on<Startup>().then([this] {
    // Make an initial Ping message to send
    auto ping = std::make_unique<Ping>();
    ping->count = 0;

    // Send the message
    emit(ping);
    });
}

}  // namespace module
