#include "Summer.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/SumInt/SumInput.hpp"
#include "message/onboarding/SumInt/SumResult.hpp"

namespace module::onboarding {

    using extension::Configuration;
    using message::onboarding::SumInt::SumInput;
    using message::onboarding::SumInt::SumResult;

    Summer::Summer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Summer.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Summer.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

    on<Trigger<SumInput>>().then([this](const SumInput& input_msg) {
            auto x = input_msg.x;
            auto y = input_msg.y;

            auto result_msg    = std::make_unique<SumResult>();
            result_msg->result = x + y;

            emit(result_msg);
    }
    }

}  // namespace module::onboarding
