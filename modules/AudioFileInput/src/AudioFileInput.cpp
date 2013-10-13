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

    const int CHUNKS_PER_SECOND = 100;

    class AudioFileInput::impl {
        public:
            SndfileHandle currentFile;
            std::vector<std::string> files;
            int currentFileIndex = 0;
            bool fileStartSent = false; //indicates whether a SoundFileStart message has been emitted to indicate the start of a new sound file
    };

    struct AudioFileConfiguration {
        static constexpr const char* CONFIGURATION_PATH = "AudioFileInput.json";
    };
    
    struct AudioFileEnd {
        int something;
    };

    AudioFileInput::AudioFileInput(NUClear::PowerPlant* plant) : Reactor(plant) {
        // Load our file name configuration.
        on<Trigger<messages::Configuration<AudioFileConfiguration>>>([this](const messages::Configuration<AudioFileConfiguration>& configfile) {
                //m->files = configfile.config["files"]; //This doesn't work for some reason
                std::vector<std::string> files1 = configfile.config["files"];
                m->files = files1;
                log("Loading sound file: ", m->files[0]);
                m->currentFile = SndfileHandle(m->files[0].c_str());
                m->fileStartSent = false;

                auto settings = std::make_unique<messages::SoundChunkSettings>();

                settings->sampleRate = m->currentFile.samplerate();
                settings->channels = m->currentFile.channels();

                settings->chunkSize = m->currentFile.samplerate() / CHUNKS_PER_SECOND;

                emit<Scope::DIRECT>(std::move(settings));
                
        });

        on<Trigger<Every<(NUClear::clock::period::den / CHUNKS_PER_SECOND), NUClear::clock::duration>>>([this](const time_t&) {
            auto& file = m->currentFile;

            auto chunk = std::make_unique<messages::SoundChunk>();

            // Find out how much of our file to read to get our sample
            size_t chunkSize = (file.samplerate() / CHUNKS_PER_SECOND) * file.channels();
            
            if (m->fileStartSent == false)
            {
                //Measures the start time of the Sound File
                auto audioStartTime = std::make_unique<messages::SoundFileStart>(); 
                audioStartTime->time = NUClear::clock::now() - NUClear::clock::duration(int(NUClear::clock::period::den * (chunkSize / m->currentFile.samplerate())));
                std::cout << "Index: " << m->currentFileIndex << std::endl;
                audioStartTime->fileName = m->files[m->currentFileIndex];
                emit(std::move(audioStartTime));
                m->fileStartSent++;
            }

            chunk->data.resize(chunkSize);
            int framesRead = file.read(chunk->data.data(), chunkSize);
            chunk->endTime = NUClear::clock::now();
            emit(std::move(chunk));
            
            if (framesRead < (int) chunkSize) //reached end of file
            {
                auto audioFileEnd = std::make_unique<AudioFileEnd>(); 
                emit(std::move(audioFileEnd));
            }
        });
        
        on<Trigger<AudioFileEnd>, Options<Single>>([this](const AudioFileEnd& audioFileEnd) {

                if (m->currentFileIndex + 1 < (int) m->files.size())
                {
                    std::cout << "reached end of file: " << m->files[m->currentFileIndex] << std::endl;
                    m->currentFileIndex++;
                    
                    m->currentFile = SndfileHandle(m->files[m->currentFileIndex].c_str());
                    
                    auto settings = std::make_unique<messages::SoundChunkSettings>();

                    settings->sampleRate = m->currentFile.samplerate();
                    settings->channels = m->currentFile.channels();

                    settings->chunkSize = m->currentFile.samplerate() / CHUNKS_PER_SECOND;

                    emit<Scope::DIRECT>(std::move(settings));
          
                    m->fileStartSent = false; //makes sure on the next trigger, a SoundFileStart message will be emitted to indicate the start time of the new sound file
                    
                }
        });
    }
}
