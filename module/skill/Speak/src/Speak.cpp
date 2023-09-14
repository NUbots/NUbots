#include "Speak.hpp"

#include <espeak/speak_lib.h>
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
            espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 500, nullptr, 1 << 15);
            espeak_SetVoiceByName("default");
            espeak_SetParameter(espeakVOLUME, 100, 0);
            espeak_SetParameter(espeakCAPITALS, 6, 0);

            emit<Task>(std::make_unique<SpeakTask>("Hello World."));
        });

        on<Provide<SpeakTask>>().then([this](const SpeakTask& speak) {
            // Wait to finish the current message (if any)
            // By waiting here this reaction can finish and return to the pool
            // if it does not have to wait for another say message
            espeak_Synchronize();

            // Say the new message
            espeak_Synth(speak.text.c_str(),     // Text
                         speak.text.size() + 1,  // Size (including null at end)
                         0,                      // Start position
                         POS_CHARACTER,          // Position Type (irrelevant since we start at the beginning)
                         0,                      // End position (0 means no end position)
                         espeakCHARS_AUTO,       // Flags (auto encoding)
                         nullptr,                // User identifier for callback
                         nullptr                 // Callback
            );
        });

        on<Shutdown>().then(espeak_Terminate);
    }

}  // namespace module::skill
