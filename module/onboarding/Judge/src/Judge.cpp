#include "Judge.hpp"

#include "extension/Configuration.hpp"
#include "message/onboarding/FinalAnswer.hpp"
#include "message/skill/Say.hpp"

namespace module::onboarding {

using extension::Configuration;

Judge::Judge(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {


    using message::skill::Say;
    using message::onboarding::FinalAnswer;


    on<Configuration>("Judge.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Judge.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
    on<Trigger<FinalAnswer>>().then([this](const FinalAnswer& answer) {
        // The correct answer for sum from k=1 to n=10 is 55
        const uint32_t EXPECTED_ANSWER = 55;

        log<INFO>("Received final answer:", answer.result);

        if (answer.result == EXPECTED_ANSWER) {
            log<INFO>("Answer is correct! Making robot nod yes.");

            // Create Say message with nod enabled
            auto say_msg = std::make_unique<Say>();
            say_msg->text = "Correct!";
            say_msg->nod = true;

            emit<Task>(say_msg);

        }
        else {
            log<WARN>("Answer is incorrect. Expected:", EXPECTED_ANSWER, "Got:", answer.result);

            // Optionally shake head or indicate wrong answer
            auto say_msg = std::make_unique<Say>();
            say_msg->text = "Incorrect.";
            say_msg->nod = false;

            emit(say_msg);
        }
    });
}

}  // namespace module::onboarding
