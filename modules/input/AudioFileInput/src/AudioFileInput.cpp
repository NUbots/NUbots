/*
 * This file is part of AudioFileInput.
 *
 * AudioFileInput is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * AudioFileInput is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with AudioFileInput.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "AudioFileInput.h"
#include "utility/idiom/pimpl_impl.h"

#include "messages/input/SoundChunk.h"
#include "messages/support/Configuration.h"
#include <chrono>
#include <string>
#include <sndfile.hh>

namespace modules {
    namespace input {

        const int CHUNKS_PER_SECOND = 100;

        class AudioFileInput::impl {
            public:
                SndfileHandle file;
        };

        struct AudioFileConfiguration {
            static constexpr const char* CONFIGURATION_PATH = "AudioFileInput.json";
        };

        AudioFileInput::AudioFileInput(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            // Load our file name configuration.
            on<Trigger<messages::support::Configuration<AudioFileConfiguration>>>([this](const messages::support::Configuration<AudioFileConfiguration>& configfile) {
                    std::string filePath = configfile.config["file"];
                    log<NUClear::DEBUG>("Loading sound file: ", filePath);
                    m->file = SndfileHandle(filePath.c_str());

                    auto settings = std::make_unique<messages::input::SoundChunkSettings>();

                    settings->sampleRate = m->file.samplerate();
                    settings->channels = m->file.channels();

                    settings->chunkSize = m->file.samplerate() / CHUNKS_PER_SECOND;

                    emit<Scope::INITIALIZE>(std::move(settings));
            });

            on<Trigger<Every<(NUClear::clock::period::den / CHUNKS_PER_SECOND), NUClear::clock::duration>>>([this](const time_t&) {
                auto& file = m->file;

                auto chunk = std::make_unique<messages::input::SoundChunk>();

                // Find out how much of our file to read to get our sample
                size_t chunkSize = (file.samplerate() / CHUNKS_PER_SECOND) * file.channels();

                chunk->data.resize(chunkSize);
                file.read(chunk->data.data(), chunkSize);
                chunk->endTime = NUClear::clock::now();
                emit(std::move(chunk));
            });
        }
        
    }  // input
}  // modules
