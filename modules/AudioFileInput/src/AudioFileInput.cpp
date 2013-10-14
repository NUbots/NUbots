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
            SndfileHandle file;
            std::string fileName;
            int currentFileIndex = 0;
            bool fileStartSent = false; //indicates whether a SoundFileStart message has been emitted to indicate the start of a new sound file
    };
    
    struct AudioFileConfiguration {
        static constexpr const char* CONFIGURATION_PATH = "AudioFileInput.json";
    };

    

    AudioFileInput::AudioFileInput(NUClear::PowerPlant* plant) : Reactor(plant) {
        // Load our file name configuration.
        on<Trigger<CommandLineArguments>>([this](const std::vector<std::string>& args) {
            m->fileName = args[args.size() - 1];//configfile.config["file"]; //args[0] is the command to run the file... so args[1] is the arg we want
            std::cout << "reading file: " << m->fileName << std::endl;
            log("Loading sound file: ", m->fileName);
            m->file = SndfileHandle(m->fileName.c_str());


            auto settings = std::make_unique<messages::SoundChunkSettings>();

            settings->sampleRate = m->file.samplerate();
            settings->channels = m->file.channels();

            settings->chunkSize = m->file.samplerate() / CHUNKS_PER_SECOND;

            //emit<Scope::DIRECT>(std::move(settings));
            emit(std::move(settings));
        });

        on<Trigger<Every<(NUClear::clock::period::den / CHUNKS_PER_SECOND), NUClear::clock::duration>>>([this](const time_t&) {
            auto& file = m->file;

            auto chunk = std::make_unique<messages::SoundChunk>();

            // Find out how much of our file to read to get our sample
            size_t chunkSize = (file.samplerate() / CHUNKS_PER_SECOND) * file.channels();
            
                        
            if (m->fileStartSent == false)
            {
                m->fileStartSent = true;
                //Measures the start time of the Sound File
                auto audioStartTime = std::make_unique<messages::SoundFileStart>(); 
                audioStartTime->time = NUClear::clock::now() - NUClear::clock::duration(int(NUClear::clock::period::den * (chunkSize / m->file.samplerate())));
                audioStartTime->fileName = m->fileName;//m->file[m->currentFileIndex];
                emit(std::move(audioStartTime));
            }

            chunk->data.resize(chunkSize);
            
            int framesRead = file.read(chunk->data.data(), chunkSize);
            chunk->endTime = NUClear::clock::now();
            emit(std::move(chunk));
            
            if (framesRead < (int) chunkSize) //reached end of file
            {
                std::cout << "end of file: " << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //lets the last chunk finish before shutting down
                powerPlant->shutdown();
                std::this_thread::sleep_for(std::chrono::milliseconds(2000)); //lets the last chunk finish before shutting down
                exit(1);
            }
            
        });

    }
}
