#include "StartCounting.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/StartCounting.hpp"

namespace module::onboarding {

using extension::Configuration;

StartCounting::StartCounting(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    using message::onboarding::StartCounting;

    on<Configuration>("StartCounting.yaml").then([this](const Configuration& config) {
        // Use configuration here from file StartCounting.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });

    on<Startup>().then([this] {
        // Start the counting chain
        auto start_counting_msg = std::make_unique<StartCounting>();

        start_counting_msg->total_iterations = 10;
        start_counting_msg->starting_sum = 0;

        emit(start_counting_msg);
    });
}

}  // namespace module::onboarding
