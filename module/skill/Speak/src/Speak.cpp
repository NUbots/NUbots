#include "Speak.hpp"

#include <cstdlib>
#include <string>

#include "extension/Configuration.hpp"

#include "message/skill/Speak.hpp"

namespace module::skill {

    using extension::Configuration;
    using SpeakTask = message::skill::Speak;

    Speak::Speak(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Speak.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Speak.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            emit<Task>(std::make_unique<SpeakTask>(std::string("Speaking is enabled.")));
        });

        on<Provide<SpeakTask>>().then([this](const SpeakTask& speak) {
            // Play the requested audio using python command-line tool mimic3 and aplay
            system(std::string("mimic3 '" + speak.text + "' | aplay").c_str());
        });
    }

}  // namespace module::skill
