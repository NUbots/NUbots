#include "Judge.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Judge.hpp"
#include "message/actuation/Limbs.hpp"

#include "utility/skill/Script.hpp"

namespace module::onboarding {

using extension::Configuration;
using extension::behaviour::Task;
using message::actuation::LimbsSequence;
using utility::skill::load_script;

Judge::Judge(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
    using message::onboarding::Judge;

    on<Configuration>("Judge.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Judge.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
        cfg.answer = config["answer"].as<int>();
    });

    on<Trigger<Judge>>().then([this](const Judge& judge_msg){
        log<NUClear::INFO>(judge_msg.val);

        if (judge_msg.val == cfg.answer){
            emit<Task>(load_script<LimbsSequence>("Stand.yaml"));
            log<NUClear::INFO>("Emit success");
        }
    });

}

}  // namespace module::onboarding
