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


static const int NUM_CHUNKS = 1;


namespace modules {
    BeatDetector::BeatDetector(NUClear::PowerPlant* plant) : Reactor(plant) {
        
        on<Trigger<Last<NUM_CHUNKS, messages::SoundChunk>>> ([this](const std::vector<std::shared_ptr<const messages::SoundChunk>>& chunks) {
            // The last NUM_CHUNKS (50) chunks of sound data that were recorded (or from a file) are received
            
            const int WINDOW_SIZE = 1024;
            const int HOP_SIZE = 512; //number of frames to input to beat detection at one time
            
            std::vector<int16_t> chunk;
            //The audio chunks are combined into a single vector
            for (int i = 0; i < NUM_CHUNKS; ++i)
            {
                chunk.insert( chunk.end(), chunks.at(i)->data.begin(), chunks.at(i)->data.end() );
            }
            
            //std::cout << "chunk size: " << chunk.size() << "\n";
            
            int numChannels = chunks.at(0)->numChannels;
            //Calculate length of recording in milliseconds
            int numMilliseconds = (int) (chunk.size() + 0.0/ numChannels) / chunks.at(0)->sampleRate * 1000 ;
            NUClear::clock::time_point endTime = chunks.at(0)->endTime;

            std::vector<int> results;
            int numBeats = 0;
            fvec_t * out;
            out = new_fvec(HOP_SIZE , numChannels);
            fvec_t * beat_input_data;
            aubio_beattracking_t * abt;
            
            std::string test = "";
            
            //For each 512 frame chunk run the beat detection algorithm (it remembers the results of the previous section)
            for (int i = 0; i < (int) chunk.size() / WINDOW_SIZE; ++i)
            {
                test = "";
                //Audio data is written to a fvec
                beat_input_data = new_fvec(HOP_SIZE , numChannels);
                for (int j = 0; j < HOP_SIZE; ++j)
                {
                   for (int k = 0; k < numChannels; ++k)
                   {
                       fvec_write_sample(beat_input_data,  chunk.at(WINDOW_SIZE* i + 2*j + k), k, j); //for channel k
                       //test += chunk.at(WINDOW_SIZE* i + 2*j + k) + " ";
                   }
                }
                
                //std::cout << "input: " << test << "\n";
                
                abt = new_aubio_beattracking(WINDOW_SIZE, numChannels);

                //out = new_fvec(WINDOW_SIZE , numChannels);

                aubio_beattracking_do(abt, beat_input_data, out);     
                
                numBeats += out->data[0][0];
                for (int j = 1; j <= out->data[0][0] -1; ++j)
                {
                    results.push_back(out->data[0][j] /** 2 + HOP_SIZE * i*/);
                }
            }


            //del_aubio_beattracking(abt);

            //Display the results of the beat detection algorithm
            std::cout << "found " << (numBeats) << " beats\n";
            
            for (int i = 0; i < results.size(); ++i)
            {
                std::cout << "Beat value " << i << " : " << (0.0 + results.at(i))/(chunk.size()/ numChannels) * numMilliseconds/1000 << "s \n";
                std::cout << "Beat value " << i << " : " << "Position in array: " << (results.at(i)) << " Size of array: " <<(chunk.size()/numChannels) << " # Seconds of recording: " << numMilliseconds/1000 << "s \n";
            }
            
            auto beatLocations = std::make_unique<messages::BeatLocations>();
            int beatPeriodFrames;
            float beatPeriodMilliseconds;
            //int numBeats;

            /*
            numBeats = out->data[0][0];
            //beatPeriod = (out->data[0][1]*4)/(chunk.data.size()/NUM_CHANNELS) * (NUM_SECONDS - out->data[0][2]*4)/(chunk.data.size()/NUM_CHANNELS) * NUM_SECONDS;
            beatPeriodFrames = out->data[0][1] - out->data[0][2];
            beatPeriodMilliseconds = beatPeriodFrames/chunk.size() * numMilliseconds ; //NUM_CHANNELS
            float firstBeatTimeMilliseconds = out->data[0][1]/chunk.size()/numChannels * numMilliseconds;
            
            NUClear::clock::time_point firstBeatTime = endTime - std::chrono::milliseconds(numMilliseconds) +  std::chrono::microseconds((int) (firstBeatTimeMilliseconds * 1000));
            
            std::time_t t = std::chrono::system_clock::to_time_t(firstBeatTime);
            std::cout <<"First beat time: " << std::ctime(&t) << "\n";
            
            beatLocations->firstBeatTime = firstBeatTime;
            beatLocations->beatPeriod = std::chrono::milliseconds( (int) beatPeriodMilliseconds);
            beatLocations->numBeats = numBeats;

            powerPlant->emit(std::move(beatLocations));
             */
        });    
    
    }

}