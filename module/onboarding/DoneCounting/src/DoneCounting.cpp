#include "DoneCounting.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/DoneCounting.hpp"
#include "message/actuation/Limbs.hpp"

#include "utility/skill/Script.hpp"

namespace module::onboarding {

using extension::Configuration;
using message::actuation::BodySequence;
using utility::skill::load_script;

DoneCounting::DoneCounting(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

    using message::onboarding::DoneCounting;

    on<Configuration>("DoneCounting.yaml").then([this](const Configuration& config) {
        // Use configuration here from file DoneCounting.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });

    on<Trigger<DoneCounting>>().then([this](const DoneCounting& done_counting_msg) {
        int sum = done_counting_msg.final_sum;
        
        log<NUClear::INFO>("Received DoneCounting message: ", std::to_string(sum));

        if(sum == 55) {
            log<NUClear::INFO>("The sum is correct! Nodding yes.");

            emit<Task>(load_script<BodySequence>("NodYes.yaml"));
        } else {
            log<NUClear::ERROR>("The sum is incorrect! Hanging head in shame.");

            emit<Task>(load_script<BodySequence>("DiveRight.yaml"));
            emit<Task>(load_script<BodySequence>("DiveLeft.yaml"));
        }
    }); 
}

}  // namespace module::onboarding
