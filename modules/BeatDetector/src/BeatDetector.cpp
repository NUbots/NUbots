/*
 * This file is part of BeatDetector.
 *
 * AudioInput is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * BeatDetector is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with BeatDetector.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Joshua Kearns <joshau-k@hotmail.com>
 */

#include "BeatDetector.h"
#include <aubio/aubio.h>
#include <chrono>
#include <ctime>
#include "messages/SoundChunk.h"
#include "messages/BeatLocations.h"
#include "tempo.h";


static const int NUM_CHUNKS = 50;


namespace modules {
    BeatDetector::BeatDetector(NUClear::PowerPlant* plant) : Reactor(plant) {
        
        
        //struct SoundChunk& chunk;
    
        //on<Trigger<messages::SoundChunk>>([this](const messages::SoundChunk& chunk) {
        on<Trigger<Last<NUM_CHUNKS, messages::SoundChunk>>> ([this](const std::vector<std::shared_ptr<const messages::SoundChunk>>& chunks) {
            //std::cout << "Beat Detector Called";
            
            
            //auto chunk = std::make_unique<messages::SoundChunk>();
            std::vector<int16_t> chunk;
            for (int i = 0; i < NUM_CHUNKS; ++i)
            {
                chunk.insert( chunk.end(), chunks.at(i)->data.begin(), chunks.at(i)->data.end() );
            }
            
            int numChannels = chunks.at(0)->numChannels;
            int numMilliseconds = (int) (chunk.size() + 0.0/ numChannels) / chunks.at(0)->sampleRate * 1000 ;
            //std::cout << "chunk size: " << chunk.size() << " num channels: " << numChannels << " sample rate: " << chunks.at(0)->sampleRate << "\n";
            //std::cout << "Length of recording: " << numMilliseconds << "ms\n";
            NUClear::clock::time_point endTime = chunks.at(0)->endTime;

            //int NUM_CHANNELS = 2;
            //int NUM_MILLISECONDS = 2;//(std::chrono::milliseconds) 2;
            fvec_t * out = new_fvec(2,numChannels);
            fvec_t * tempo_input_data;
            //struct messages::BeatLocations * beatLocations;
            
            //messages::BeatLocations * beatLocations = new messages::BeatLocations;
           auto beatLocations = std::make_unique<messages::BeatLocations>();
            
            tempo_input_data = new_fvec(chunk.size()/numChannels , numChannels);

           for (int i = 0; i < (int) chunk.size()/numChannels; ++i)
            {
               for (int j = 0; j < numChannels; ++j)
               {
                   fvec_write_sample(tempo_input_data,  chunk.at(2*i + j), j, i); //for channel j
               }
                //fvec_write_sample(beat_input_data,  chunk.at(2*i), 0, i);
                //fvec_write_sample(beat_input_data, chunk.at(2*i + 1), 1, i);
            }
            
              int buffer_size = chunk.size()/numChannels;//1024;
              int overlap_size = 512;
            
            aubio_tempo_t * att = new_aubio_tempo (aubio_onset_kl, buffer_size, overlap_size, numChannels);
 
            aubio_tempo(att, tempo_input_data, out);
            
            
            
            for (int i = 0; i <= out->data[0][0] -1; ++i)
            {
                std::cout << "out->data:i= " << out->data[0][i] <<"\n";
                //std::cout << "Beat value " << i << " : " << (out->data[0][i])/(chunk.size()/ numChannels) * numMilliseconds/1000 << "s \n";
                //std::cout << "Beat value " << i << " : " << "Position in array: " << (out->data[0][i]) << " Size of array: " <<(chunk.size()/numChannels) << " # Seconds of recording: " << numMilliseconds/1000 << "s \n";

            }
            
            del_aubio_tempo(att);
            
            /*
            abt = new_aubio_beattracking(chunk.size()/numChannels, numChannels);
            //abt = new_aubio_beattracking(sizeof(SAMPLE), NUM_CHANNELS);

            out = new_fvec(chunk.size()/numChannels , numChannels);

            aubio_beattracking_do(abt, beat_input_data, out);

            del_aubio_beattracking(abt);

            std::cout << "found " << (out->data[0][0] -1) << " beats\n";
            
            //beatFound = false;
            //start = 0;
            //end = 0;
            for (int i = 1; i <= out->data[0][0] -1; ++i)
            {
                
                std::cout << "Beat value " << i << " : " << (out->data[0][i])/(chunk.size()/ numChannels) * numMilliseconds/1000 << "s \n";
                std::cout << "Beat value " << i << " : " << "Position in array: " << (out->data[0][i]) << " Size of array: " <<(chunk.size()/numChannels) << " # Seconds of recording: " << numMilliseconds/1000 << "s \n";

            }

            numBeats = out->data[0][0];
            //beatPeriod = (out->data[0][1]*4)/(chunk.data.size()/NUM_CHANNELS) * (NUM_SECONDS - out->data[0][2]*4)/(chunk.data.size()/NUM_CHANNELS) * NUM_SECONDS;
            beatPeriodFrames = out->data[0][1] - out->data[0][2];
            beatPeriodMilliseconds = beatPeriodFrames/chunk.size() * numMilliseconds ; //NUM_CHANNELS
            float firstBeatTimeMilliseconds = out->data[0][1]/chunk.size()/numChannels * numMilliseconds;
            
            //std::time_t firstBeatTime = std::chrono::system_clock::to_time_t(chunk.endTime - std::chrono::milliseconds(NUM_MILLISECONDS) +  std::chrono::microseconds((int) (firstBeatTimeMilliseconds * 1000)));
            //NUClear::clock::time_point firstBeatTimePoint = std::chrono::system_clock::from_time_t(firstBeatTime);
            
            NUClear::clock::time_point firstBeatTime = endTime - std::chrono::milliseconds(numMilliseconds) +  std::chrono::microseconds((int) (firstBeatTimeMilliseconds * 1000));
            
            std::time_t t = std::chrono::system_clock::to_time_t(firstBeatTime);
            //std::cout << 
            std::cout <<"First beat time: " << std::ctime(&t) << "\n";
            */
            //beatLocations->firstBeatTime = firstBeatTime;
            //beatLocations->beatPeriod = std::chrono::milliseconds( (int) beatPeriodMilliseconds);
            //beatLocations->numBeats = numBeats;

            //powerPlant->emit(std::move(beatLocations));
            //powerPlant->emit(std::move(chunk));
             
        });    
    
    }

}