#include "WavReader.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

#include "extension/Configuration.hpp"
namespace module::input {

    using extension::Configuration;
    
    //function to read wav file - See example of functions in
    
    WavReader::WavReader(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("WavReader.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file WavReader.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
            this->log_level = cfg["wav_path"].as<std::string>>();
        });
    }

}  // namespace module::input
