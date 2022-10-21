#include "WavReader.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>

#include "extension/Configuration.hpp"

#include "message/input/AudioData.hpp"
#include "message/support/GlobalConfig.hpp"

namespace module::input {

    using extension::Configuration;
    using message::input::AudioData;

    WavReader::WavReader(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        // uncomment if you want to check if the path exists.
        // bool filepathExists =
        // std::filesystem::exists("/home/nubots/NUbots/module/input/WavReader/data/audio/test.wav");; std::cout <<
        // filepathExists << std::endl;


        on<Configuration>("WavReader.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file WavReader.yaml

            this->log_level       = cfg["log_level"].as<NUClear::LogLevel>();
            this->config.wav_path = cfg["wav_path"].as<std::string>();  // config is the name of the struct
            std::cout << config.wav_path << std::endl;
            log<NUClear::DEBUG>(config.wav_path);
        });

        // This reactor receives audiodata from emission
        // on<Trigger<AudioData>>().then([this] (AudioData& audioData ) {
        //  reactor code
        //});

        on<Startup>().then([this]() { readWav(); });
    }

    void WavReader::readWav() {
        FILE* wavin;
        uint32_t nread = 0;
        uint32_t size  = 0;

        wavin = fopen(config.wav_path.c_str(), "rb");
        fseek(wavin, 40, SEEK_SET);
        fread(&size, sizeof(uint32_t), 1, wavin);
        log<NUClear::DEBUG>(size);

        // Make it one longer for the null termination
        auto audioData     = std::make_unique<AudioData>();
        audioData->WavData = std::vector<uint8_t>(size);

        nread = fread(audioData->WavData.data(), sizeof(uint8_t), size, wavin);

        if (nread != size) {
            // TODO(Tom Legge) don't throw
            throw std::invalid_argument("number read from wav file was not its size");
        }

        fclose(wavin);


        emit(audioData);
    }

}  // namespace module::input
