#include "Judge.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/onboarding/FinalAnswer.hpp"

#include "utility/skill/Script.hpp"

namespace module::onboarding {

using extension::Configuration;
using message::actuation::HeadSequence;
using message::onboarding::FinalAnswer;
using utility::skill::load_script;

Judge::Judge(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {


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

            // Load and emit head nod sequence
            emit<Task>(load_script<HeadSequence>("NodYes.yaml"));
        }
        else {
            log<WARN>("Answer is incorrect. Expected:", EXPECTED_ANSWER, "Got:", answer.result);
        }
    });
}

}  // namespace module::onboarding
