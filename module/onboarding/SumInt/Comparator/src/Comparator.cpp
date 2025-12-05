#include "Comparator.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/SumInt/FinalResult.hpp"
#include "message/onboarding/SumInt/SumInput.hpp"
#include "message/onboarding/SumInt/SumResult.hpp"

namespace module::onboarding::SumInt {

    using extension::Configuration;
    using message::onboarding::SumInt::FinalResult;
    using message::onboarding::SumInt::SumInput;
    using message::onboarding::SumInt::SumResult;

    Comparator::Comparator(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        this->n = 10;
        this->k = 1;

        // import config options
        on<Configuration>("Comparator.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            this->n         = config["n"].as<int>();
            this->k         = config["k"].as<int>();
        });

        // on startup emit first sum input message
        on<Startup>().then([this] {
            auto sum_input = std::make_unique<SumInput>();
            sum_input->x   = 0;
            sum_input->y   = k;

            log<INFO>("Emitting SumInput{", sum_input->x, ",", sum_input->y, "}");

            emit(sum_input);
        });

        // consume sum result message and process:
        // trigger another sum or if done, emit final result message
        on<Trigger<SumResult>>().then([this](const SumResult& sum_result) {
            auto result = sum_result.result;
            log<INFO>("Received SumResult{", result, "}");

            k++;
            // finished, emit final result message
            if (k > n) {
                auto final_result    = std::make_unique<FinalResult>();
                final_result->result = result;

                log<INFO>("Emitting FinalResult{", final_result->result, "}");

                emit(final_result);

                return;
            }

            auto sum_input = std::make_unique<SumInput>();
            sum_input->x   = result;
            sum_input->y   = k;

            log<INFO>("Emitting SumInput{", sum_input->x, ",", sum_input->y, "}");

            emit(sum_input);
        });
    }

}  // namespace module::onboarding::SumInt
