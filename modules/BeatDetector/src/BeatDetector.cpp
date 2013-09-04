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




namespace modules {
    BeatDetector::BeatDetector(NUClear::PowerPlant* plant) : Reactor(plant) {
        //struct SoundChunk& chunk;
    
        on<Trigger<messages::SoundChunk>>([this](const messages::SoundChunk& chunk) {

            int NUM_CHANNELS = 2;
            int NUM_MILLISECONDS = 2;//(std::chrono::milliseconds) 2;
            fvec_t * out;
            fvec_t * beat_input_data;
            aubio_beattracking_t * abt;
            //struct messages::BeatLocations * beatLocations;
            
            //messages::BeatLocations * beatLocations = new messages::BeatLocations;
           auto beatLocations = std::make_unique<messages::BeatLocations>();
            
            //NUClear::clock::time_point firstBeatTime;
            int beatPeriodFrames;
            float beatPeriodMilliseconds;
            //NUClear::clock::duration beatPeriod;
            int numBeats;
            //NUClear::clock::time_point firstBeatTime;
            
            
            beat_input_data = new_fvec(chunk.data.size()/NUM_CHANNELS , NUM_CHANNELS);
            //new_fvec();
                        //data.recordedSamples;
           chunk.data.at(0);


           for (int i = 0; i < (int) chunk.data.size()/2; ++i)
            {
                fvec_write_sample(beat_input_data,  chunk.data.at(2*i), 0, i);
                fvec_write_sample(beat_input_data, chunk.data.at(2*i + 1), 1, i);
                //fvec_write_sample(beat_input_data,  data.recordedSamples[2*i], 0, i);
                //fvec_write_sample(beat_input_data, data.recordedSamples[2*i + 1], 1, i);
            }
 
            abt = new_aubio_beattracking(chunk.data.size()/NUM_CHANNELS, NUM_CHANNELS);
            //abt = new_aubio_beattracking(sizeof(SAMPLE), NUM_CHANNELS);

            out = new_fvec(chunk.data.size()/NUM_CHANNELS , NUM_CHANNELS);

            aubio_beattracking_do(abt, beat_input_data, out);

            del_aubio_beattracking(abt);

            std::cout << "found " << (out->data[0][0] -1) << " beats\n";
            
            //beatFound = false;
            //start = 0;
            //end = 0;
            for (int i = 1; i <= out->data[0][0] -1; ++i)
            {
                std::cout << "Beat value " << i << " : " << (out->data[0][i]*4)/(chunk.data.size()/NUM_CHANNELS) * NUM_MILLISECONDS/1000 << "s \n";

            }

            numBeats = out->data[0][0];
            //beatPeriod = (out->data[0][1]*4)/(chunk.data.size()/NUM_CHANNELS) * (NUM_SECONDS - out->data[0][2]*4)/(chunk.data.size()/NUM_CHANNELS) * NUM_SECONDS;
            beatPeriodFrames = out->data[0][1] - out->data[0][2];
            beatPeriodMilliseconds = beatPeriodFrames/chunk.data.size() * NUM_MILLISECONDS ; /*/NUM_CHANNELS*/
            float firstBeatTimeMilliseconds = out->data[0][1]/chunk.data.size()/NUM_CHANNELS * NUM_MILLISECONDS;
            
            //std::time_t firstBeatTime = std::chrono::system_clock::to_time_t(chunk.endTime - std::chrono::milliseconds(NUM_MILLISECONDS) +  std::chrono::microseconds((int) (firstBeatTimeMilliseconds * 1000)));
            //NUClear::clock::time_point firstBeatTimePoint = std::chrono::system_clock::from_time_t(firstBeatTime);
            
            NUClear::clock::time_point firstBeatTime = chunk.endTime - std::chrono::milliseconds(NUM_MILLISECONDS) +  std::chrono::microseconds((int) (firstBeatTimeMilliseconds * 1000));
            
            beatLocations->firstBeatTime = firstBeatTime;
            beatLocations->beatPeriod = std::chrono::milliseconds( (int) beatPeriodMilliseconds);
            beatLocations->numBeats = numBeats;

            powerPlant->emit(std::move(beatLocations));
            //powerPlant->emit(std::move(chunk));
             
        });    
    
    }

}