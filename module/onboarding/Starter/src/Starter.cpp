#include "Starter.hpp"

#include "extension/Configuration.hpp"
#include "message/onboarding/ComputeIteration.hpp"


namespace module::onboarding {

using extension::Configuration;



Starter::Starter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    using message::onboarding::ComputeIteration;

    on<Configuration>("Starter.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Starter.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
    on<Startup>().then([this] {
        log<INFO>("Starting the summation computation...");

        // Create the first iteration message
        // Start with k=1 (first number to add) and sum=0 (nothing added yet)
        auto compute_msg = std::make_unique<ComputeIteration>();
        compute_msg->k = 1;      // Start with k=1
        compute_msg->sum = 0;    // Initial sum is 0

        log<INFO>("Emitting ComputeIteration with k=1, sum=0");
        emit(compute_msg);
    });
}

}  // namespace module::onboarding
