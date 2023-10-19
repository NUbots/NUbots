#include "Say.hpp"

#include <cstdlib>
#include <string>
#include <thread>

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/skill/Say.hpp"

#include "utility/skill/Script.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::LimbsSequence;
    using utility::skill::load_script;
    using SayTask = message::skill::Say;

    Say::Say(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Say.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Say.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.voice       = config["voice"].as<std::string>();
        });

        on<Provide<SayTask>>().then([this](const SayTask& say, const RunInfo& info) {
            // Only say text if it is a new task
            if (info.run_reason == RunInfo::NEW_TASK) {
                // Play the requested audio using python command-line tool mimic3 and aplay
                // Sanitize the text to remove special characters which could break the command
                std::string sanitized_text         = say.text;
                const std::string chars_to_replace = "'\"`|;&$\\";
                for (char c : chars_to_replace) {
                    std::replace(sanitized_text.begin(),
                                 sanitized_text.end(),
                                 c,
                                 ' ');  // Replacing each dangerous char with space
                }
                log<NUClear::DEBUG>("Saying: ", sanitized_text);
                system(std::string("mimic3 '" + sanitized_text + "' --voice '" + cfg.voice + "' | aplay").c_str());
            }

            // Nod head to indicate that the robot is "talking"
            if (say.nod) {
                emit<Task>(load_script<LimbsSequence>("NodYes.yaml"));
            }
        });
    }

}  // namespace module::skill
