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
#include <sndfile.hh>
#include "messages/SoundChunk.h"
#include <vector>
#include <iostream>
#include <chrono>


namespace modules {
    AudioFileInput::AudioFileInput(NUClear::PowerPlant* plant) : Reactor(plant) {
        //struct SoundChunk& chunk;
        
        //on<Trigger<Every<10000, std::chrono::milliseconds>>> ([this](const time_t&) {
        on<Trigger<Every<100, std::chrono::milliseconds>>, Options<Single>> ([this](const time_t&) {
            //every 100 milliseconds it will check if it is not already running, if it isn't it will start a new one
            
            
            const int NUM_FRAMES = 15000000; //if this is too big it will cause a segmentation fault. 300000000 is certainly too big
            const int FRAME_RATE = 44000;
            const int NUM_CHANNELS = 2;
            const int INTS_PER_FRAME = 2;
            const float CHUNK_TIME = 0.1; //how long each chunk is in seconds

            int framesPerChunk = (int) FRAME_RATE * CHUNK_TIME;

            sf_count_t num_frames = (sf_count_t) NUM_FRAMES;

            SF_INFO  info = {num_frames, FRAME_RATE, NUM_CHANNELS, SF_FORMAT_WAV, 1, 1};
            
            
            std::string loc = "recorded.wav";//"/NuclearPort2Shortcut/NUClearPort2/recorded.wav";
            SNDFILE* sndfile = sf_open(loc.c_str(), SFM_READ, &info);
            
            short* frames = new short[NUM_FRAMES];//short frames[NUM_FRAMES];// = short[NUM_FRAMES];

            sf_count_t framesRead = sf_read_short (sndfile, frames, (sf_count_t) NUM_FRAMES) ;
            
            sf_close(sndfile);
            //std::cout << "number of frames: " << framesRead << "\n";
       
            int startPos = 0;
            
            while (startPos + framesPerChunk < framesRead)
            {
                auto chunk = std::make_unique<messages::SoundChunk>();
                chunk->data.resize(framesPerChunk); 
                chunk->data.assign(frames + startPos, frames + startPos + framesPerChunk ); //first item and last item to be read 
                startPos += framesPerChunk;

                //std::cout << "StartPos: " << startPos << "\n";
                chunk->numChannels = NUM_CHANNELS;
                chunk->sampleRate = FRAME_RATE;
                chunk->endTime = std::chrono::system_clock::now();
                emit(std::move(chunk));
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
            }
        
        });
    
    }

}
