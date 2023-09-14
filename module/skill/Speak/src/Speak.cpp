#include "Speak.hpp"

#include <cstdlib>
#include <string>
#include <thread>

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/skill/Speak.hpp"

#include "utility/skill/Script.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::LimbsSequence;
    using utility::skill::load_script;
    using SpeakTask = message::skill::Speak;

    Speak::Speak(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Speak.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Speak.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            emit<Task>(std::make_unique<SpeakTask>(std::string("Speaking is enabled.")));
        });

        on<Provide<SpeakTask>>().then([this](const SpeakTask& speak, const RunInfo& info) {
            // If it is a new task, then speak
            if (info.run_reason == RunInfo::NEW_TASK) {
                // Play the requested audio using python command-line tool mimic3 and aplay
                std::thread([speak]() { system(std::string("mimic3 '" + speak.text + "' | aplay").c_str()); }).detach();
            }

            // Nod head to indicate that the robot is "talking"
            emit<Task>(load_script<LimbsSequence>("NodYes.yaml"));
        });
    }

}  // namespace module::skill
