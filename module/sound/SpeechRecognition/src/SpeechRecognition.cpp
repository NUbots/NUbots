#include "SpeechRecognition.hpp"

#include <vosk_api.h>

#include "extension/Configuration.hpp"

#include "message/input/Audio.hpp"

namespace module::sound {

    using extension::Configuration;
    using message::input::Audio;


    SpeechRecognition::SpeechRecognition(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("SpeechRecognition.yaml").then([this](const Configuration& config) {
            // Use configuration here from file SpeechRecognition.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
            cfg.model_path  = config["model_path"].as<std::string>();
            model           = vosk_model_new(cfg.model_path);
            recognizer      = vosk_recognizer_new(model, 16000.0);
        });

        // This reactor receives audio from emission
        on<Trigger<Audio>>().then([this](const Audio& audio) {
            int final = vosk_recognizer_accept_waveform(recognizer,
                                                        reinterpret_cast<const char*>(audio.audio.data()),
                                                        audio.audio.size());

            log<NUClear::DEBUG>(audio.audio.size());
            log<NUClear::DEBUG>(final);
            log<NUClear::DEBUG>(vosk_recognizer_result(recognizer));
        });

        on<Shutdown>().then([this] {
            // Cleanup
            vosk_recognizer_free(recognizer);
            vosk_model_free(model);
        });
    }


}  // namespace module::sound
