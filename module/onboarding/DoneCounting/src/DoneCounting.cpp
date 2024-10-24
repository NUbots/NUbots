#include "DoneCounting.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/DoneCounting.hpp"

namespace module::onboarding {

using extension::Configuration;

DoneCounting::DoneCounting(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    using message::onboarding::DoneCounting;

    on<Configuration>("DoneCounting.yaml").then([this](const Configuration& config) {
        // Use configuration here from file DoneCounting.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });

    on<Trigger<DoneCounting>>().then([this](const DoneCounting& done_counting_msg) {
        log<NUClear::INFO>("Received DoneCounting message: ", std::to_string(done_counting_msg.final_sum));
    }); 
}

}  // namespace module::onboarding
