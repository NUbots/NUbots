#include "Accumulator.hpp"

#include "extension/Configuration.hpp"
#include "message/onboarding/ComputeIteration.hpp"
#include "message/onboarding/FinalAnswer.hpp"


namespace module::onboarding {

using extension::Configuration;

Accumulator::Accumulator(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    using message::onboarding::ComputeIteration;
    using message::onboarding::FinalAnswer;

    on<Configuration>("Accumulator.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Accumulator.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
    on<Trigger<ComputeIteration>>().then([this](const ComputeIteration& compute_msg) {
        // Get current values
        uint32_t k = compute_msg.k;
        uint32_t current_sum = compute_msg.sum;

        // Add k to the sum
        uint32_t new_sum = current_sum + k;

        log<INFO>("Iteration k=", k, ": sum was", current_sum, ", adding", k, ", new sum is", new_sum);

        // Maximum number of iterations (n=10)
        const uint32_t MAX_N = 10;

        // Check if we need to continue or finish
        if (k < MAX_N) {
            // Continue iterating - emit next iteration
            auto next_iteration = std::make_unique<ComputeIteration>();
            next_iteration->k = k + 1;      // Increment k
            next_iteration->sum = new_sum;   // Pass along the new sum

            log<INFO>("Continuing: emitting ComputeIteration with k=", (k + 1), ", sum=", new_sum);
            emit(next_iteration);
        }
        else {
            // We've reached k=10, time to finish
            auto final_answer = std::make_unique<FinalAnswer>();
            final_answer->result = new_sum;

            log<INFO>("Computation complete! Final answer:", new_sum);
            emit(final_answer);
        }
    });
}

}  // namespace module::onboarding
