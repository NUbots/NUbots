#include "Judge.hpp"

#include "extension/Configuration.hpp"
#include "message/onboarding/FoundAnswer.hpp"

namespace module::onboarding {

using extension::Configuration;

Judge::Judge(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {


    using message::onboarding::FoundAnswer;

    on<Configuration>("Judge.yaml").then([this](const Configuration& config) {
        // Use configuration here from file Judge.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });

    on<Trigger<FoundAnswer>>().then([this](const FoundAnswer& found_answer_msg) {

        // Once found answer is received, we can emit a task to nod yes
        auto found_answer = found_answer_msg.count + 1;

        if(found_answer == 1){
            log<INFO>("Nod Yes!");

            // Using NodYes.yaml to emit a task to nod yes
            emit<Task>(load_script("NodYes.yaml"));
        }
        else{
            log<INFO>("Shake Head!");
        }

    });

}

}  // namespace module::onboarding
