#include "Actuator.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/onboarding/SumInt/FinalResult.hpp"

#include "utility/skill/Script.hpp"

namespace module::onboarding::SumInt {

    using extension::Configuration;
    using extension::behaviour::Task;
    using message::actuation::BodySequence;
    using message::actuation::HeadSequence;
    using message::onboarding::SumInt::FinalResult;
    using utility::skill::load_script;

    Actuator::Actuator(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Actuator.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<FinalResult>>().then([this](const FinalResult& final_result) {
            auto result = final_result.result;
            log<INFO>("Received FinalResult{", result, "}");

            // emit NodYes servo sequences
            // which are loaded from the NodYes.yaml script
            log<INFO>("Emitting NodYes servo sequences");
            emit<Task>(load_script<BodySequence>("NodYes.yaml"));
        });
    }

}  // namespace module::onboarding::SumInt
