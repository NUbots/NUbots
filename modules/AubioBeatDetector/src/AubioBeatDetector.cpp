/*
 * This file is part of AubioBeatDetector.
 *
 * AudioInput is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * AubioBeatDetector is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with AubioBeatDetector.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Joshua Kearns <joshau-k@hotmail.com>
 */

#include "AubioBeatDetector.h"
#include <aubio/aubio.h>
#include <chrono>
#include <ctime>
#include "messages/SoundChunk.h"
#include "messages/Beat.h"
#include <vector>

namespace modules {

    static const int WINDOW_SIZE = 1024;
    static const int HOP_SIZE = 512; //number of frames to input to beat detection at one time


    //used to hold chunk data resized to HOP_SIZE
    struct ResizedChunk { 
            std::vector<int16_t> resizedChunk;
            NUClear::clock::time_point endTime;
    };
    
    // Holds beat value without a period, used to create a Beat objects with a period by comparing the last 2 beatTime's
    struct BeatTime {
            NUClear::clock::time_point time;
    };
        
    AubioBeatDetector::AubioBeatDetector(NUClear::PowerPlant* plant) : Reactor(plant) {
        
        //chunkWindow.reserve(HOP_SIZE); //This will be filled with HOP_SIZE (512) floats received from AudioInput or AudioFileInput then emitted as a 'resizedChunk' to be processed by aubio
        
        allowBeat = false;
        callCount = 0;
        beatCount = 0;
        
        
        on<Trigger<messages::SoundChunkSettings>>([this](const messages::SoundChunkSettings& settings) {

            // Store the settings of the sound chunks
            sampleRate = settings.sampleRate;
            channels = settings.channels;
            chunkSize = settings.chunkSize;
            allowBeat = true;
            
            
            att =  new_aubio_tempo (aubio_onset_kl, WINDOW_SIZE, HOP_SIZE, channels); //aubio_onset_kl or aubio_onset_complex (may be others too)
            out = new_fvec(HOP_SIZE , channels);
            tempo_input_data = new_fvec(WINDOW_SIZE , channels);


        });
        
        on<Trigger<messages::SoundChunk>> ([this](const messages::SoundChunk& chunk) {
            if (allowBeat == true) {
                //if (chunk.endTime == NULL) This hasn't been provided so far, but comparing to NULL doesn't work
                //chunk.endTime = NUClear::clock::now(); //also chunk is const so this doesn't work either
                
                auto outputChunk = std::make_unique<ResizedChunk>();
                int chunkWindowSize = chunkWindow.size();
                for (int i =0; i < (int) chunk.data.size(); ++i) 
                {
                    chunkWindow.push_back(chunk.data[i]);
                    ++chunkWindowSize;
                    
                    if (chunkWindowSize == HOP_SIZE)
                    {
                        
                        outputChunk->resizedChunk = chunkWindow;
                        outputChunk->endTime = NUClear::clock::now() - std::chrono::microseconds( (int) (1000000.0*(chunk.data.size() - i)/sampleRate));
                        //This ^ calculates the time the resized chunk ends given the time the chunk received by the trigger ends and how far through the received chunk we are
                        emit(std::move(outputChunk));
                        chunkWindow.clear();
                        chunkWindowSize = 0;
                        auto outputChunk = std::make_unique<ResizedChunk>();
                    }
                }
                
            }
        });
        
        on<Trigger<ResizedChunk>> ([this](const ResizedChunk& resizedChunk) {
            // The last resized chunk of size HOP_SIZE (512) of sound data that were recorded are received and processed by aubio tempo track

            std::vector<int16_t> chunk = resizedChunk.resizedChunk;
            //chunk.insert( chunk.end(), resizedChunk.resizedChunk.begin(), resizedChunk.resizedChunk.end() );

            std::vector<int> results;

            for (int i = 0; i < HOP_SIZE; ++i)
            {
                for (int j = 0; j < channels; ++j)
                {
                    fvec_write_sample(tempo_input_data,  chunk.at(channels*i + j), j, i); //for channel k
                }
            }

            auto beatTime = std::make_unique<BeatTime>();
            aubio_tempo(att, tempo_input_data, out);     
            for (int i =0; i < out->data[0][0]; ++i)
            {
                beatTime = std::make_unique<BeatTime>();
                beatTime->time = resizedChunk.endTime - std::chrono::microseconds( (int) ((HOP_SIZE - out->data[0][i] + 0.0) / sampleRate));
                //We start with the time at the end of the chunk. Beat time is found from the time at the start of the chunk plus the number of frames into the chunk that the beat is found. 
                        
                emit(std::move(beatTime));        
            }

       });    
       
       on<Trigger<Last<2, BeatTime>>> ([this](const std::vector<std::shared_ptr<const BeatTime>>& lastTwoBeats) {
           
           if(lastTwoBeats.size() > 1) {
                auto beat = std::make_unique<messages::Beat>();
                beat->time = lastTwoBeats[0]->time; //apparently the latest one is 0
                beat->period = lastTwoBeats[0]->time - lastTwoBeats[1]->time;

                emit(std::move(beat));  
           }
       }); 


        
    }
    
    AubioBeatDetector::~AubioBeatDetector()
    {
          del_aubio_tempo(att);
          del_fvec(tempo_input_data);
          del_fvec(out);
    }

}