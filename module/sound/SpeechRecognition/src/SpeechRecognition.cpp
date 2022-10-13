#include "SpeechRecognition.hpp"

#include <vosk_api.h>

#include "extension/Configuration.hpp"

namespace module {

    using extension::Configuration;
		

    SpeechRecognition::SpeechRecognition(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("SpeechRecognition.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file SpeechRecognition.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });
        
        //This reactor receives audiodata from emission
	on<Trigger<AudioData>>().then([this] (AudioData& audioData ) {
	});	
    }


}  // namespace module
