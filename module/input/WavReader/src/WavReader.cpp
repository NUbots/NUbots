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
    using message::input::Audio;

    WavReader::WavReader(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("WavReader.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WavReader.yaml

            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
            cfg.wav_path    = cfg["wav_path"].as<std::string>();

            if (!std::filesystem::exists(cfg.wav_path)) {
                log<NUClear::ERROR>("Wav file does not exist");
            }
            else {
                log<NUClear::INFO>("Reading wav file:", cfg.wav_path);
                read_wav();
            }
        });
    }

    void WavReader::read_wav() {
        FILE* wavin;
        uint32_t nread = 0;
        uint32_t size  = 0;

        wavin = fopen(cfg.wav_path.c_str(), "rb");
        fseek(wavin, 40, SEEK_SET);
        fread(&size, sizeof(uint32_t), 1, wavin);

        auto audio       = std::make_unique<Audio>();
        audio->timestamp = NUClear::clock::now();
        audio->audio     = std::vector<uint8_t>(size);

        nread = fread(audio->audio.data(), sizeof(uint8_t), size, wavin);

        if (nread != size) {
            log<NUClear::ERROR>("Expected and actual size of wav file do not match");
        }
        else {
            emit(audio);
        }

        fclose(wavin);
    }

}  // namespace module::input
