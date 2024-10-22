#include "Counting.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/StartCounting.hpp"
#include "message/onboarding/Counting.hpp"
#include "message/onboarding/DoneCounting.hpp"

namespace module::onboarding {

using extension::Configuration;

Counting::Counting(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    using message::onboarding::StartCounting;
    using message::onboarding::Counting;
    using message::onboarding::DoneCounting;

    on<Configuration>("Counting.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Counting.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });

    on<Trigger<StartCounting>>().then([this](const StartCounting& start_counting_msg) {
        auto counting_msg = std::make_unique<Counting>();

        counting_msg->iterations_left = start_counting_msg.total_iterations;
        counting_msg->sum = start_counting_msg.starting_sum;

        emit(counting_msg);
    });

    on<Trigger<Counting>>().then([this](const Counting& counting_msg) {
        if(counting_msg.iterations_left == 0) {
            auto done_counting_msg = std::make_unique<DoneCounting>();

            done_counting_msg->final_sum = counting_msg.sum;

            emit(done_counting_msg);
        } else {
            auto counting_msg_new = std::make_unique<Counting>();

            counting_msg_new->iterations_left = counting_msg.iterations_left - 1;
            counting_msg_new->sum = counting_msg.sum + counting_msg.iterations_left;

            emit(counting_msg_new);
        }
    });
}

}  // namespace module::onboarding
