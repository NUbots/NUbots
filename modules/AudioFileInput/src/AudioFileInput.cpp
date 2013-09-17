/*
 * This file is part of AudioFileInput.
 *
 * AudioFileInput is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * AudioInput is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with AudioInput.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Joshua Kearns <joshau-k@hotmail.com>
 */

//#include <algorithm>

#include "AudioFileInput.h"
#include "utility/idiom/pimpl_impl.h"

#include "messages/SoundChunk.h"
#include "messages/Configuration.h"
#include <chrono>
#include <string>
#include <sndfile.hh>

namespace modules {

    class AudioFileInput::impl {
        public:
            SndfileHandle file;
    };

    struct AudioFileConfiguration {
        static constexpr const char* CONFIGURATION_PATH = "AudioFileInput.json";
    };

    AudioFileInput::AudioFileInput(NUClear::PowerPlant* plant) : Reactor(plant) {
        // Load our file name configuration.
        on<Trigger<messages::Configuration<AudioFileConfiguration>>>([this](const messages::Configuration<AudioFileConfiguration>& configfile) {
                std::string filePath = configfile.config["file"];
                log("Loading sound file: ", filePath);
                m->file = SndfileHandle(filePath.c_str());
        });

        on<Trigger<Every<100, std::chrono::milliseconds>>>([this](const time_t&) {
            auto& file = m->file;

            auto chunk = std::make_unique<messages::SoundChunk>();

            // Find out how much of our file to read to get a 100ms sample
            size_t chunkSize = (file.samplerate() / 10) * file.channels();

            chunk->data.resize(chunkSize);
            file.read(chunk->data.data(), chunkSize);
            chunk->channels = file.channels();
            chunk->sampleRate = file.samplerate();
            emit(std::move(chunk));
        });
    }
}
