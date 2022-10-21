#include "SpeechRecognition.hpp"

#include <vosk_api.h>

#include "extension/Configuration.hpp"

#include "message/input/AudioData.hpp"

namespace module::sound {

    using extension::Configuration;
    using message::input::AudioData;


    SpeechRecognition::SpeechRecognition(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("SpeechRecognition.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file SpeechRecognition.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        // This reactor receives audiodata from emission
        on<Trigger<AudioData>>().then([this](const AudioData& audioData) {
            VoskModel* model           = vosk_model_new("/usr/local/src/vosk-api/c/model");
            VoskRecognizer* recognizer = vosk_recognizer_new(model, 16000.0);
            int final                  = vosk_recognizer_accept_waveform(recognizer,
                                                        reinterpret_cast<const char*>(audioData.WavData.data()),
                                                        audioData.WavData.size());
            log<NUClear::DEBUG>(audioData.WavData.size());
            log<NUClear::DEBUG>(final);
            log<NUClear::DEBUG>(vosk_recognizer_result(recognizer));
            vosk_recognizer_free(recognizer);
            vosk_model_free(model);
        });
    }


}  // namespace module::sound
