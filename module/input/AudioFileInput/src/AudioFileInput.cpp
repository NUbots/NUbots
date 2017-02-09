/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "AudioFileInput.h"
#include "utility/idiom/pimpl_impl.h"

#include "message/input/SoundChunk.h"
#include "extension/Configuration.h"
#include <chrono>
#include <string>
#include <sndfile.hh>

namespace module {
    namespace input {

        const int CHUNKS_PER_SECOND = 100;

        using extension::Configuration;

        class AudioFileInput::impl {
            public:
                SndfileHandle file;
        };

        AudioFileInput::AudioFileInput(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            // Load our file name configuration.
            on<Configuration>("AudioFileInput.yaml").then([this](const Configuration& configfile) {
                    std::string filePath = configfile.config["file"];
                    NUClear::log<NUClear::DEBUG>("Loading sound file: ", filePath);
                    m->file = SndfileHandle(filePath.c_str());

                    auto settings = std::make_unique<message::input::SoundChunkSettings>();

                    settings->sampleRate = m->file.samplerate();
                    settings->channels = m->file.channels();

                    settings->chunkSize = m->file.samplerate() / CHUNKS_PER_SECOND;

                    emit<Scope::INITIALIZE>(std::move(settings));
            });

            on<Every<CHUNKS_PER_SECOND, Per<std::chrono::seconds>>>([this] {
                auto& file = m->file;

                auto chunk = std::make_unique<message::input::SoundChunk>();

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
