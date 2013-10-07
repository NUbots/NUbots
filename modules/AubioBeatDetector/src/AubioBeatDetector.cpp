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


    struct resizedChunk {
            std::vector<int16_t> resizedChunk;
            NUClear::clock::time_point endTime;
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
                
                
                auto outputChunk = std::make_unique<resizedChunk>();
                int chunkWindowSize = chunkWindow.size();
                for (int i =0; i < (int) chunk.data.size(); ++i) 
                {
                    chunkWindow.push_back(chunk.data[i]);
                    ++chunkWindowSize;
                    
                    if (chunkWindowSize == HOP_SIZE)
                    {
                        
                        outputChunk->resizedChunk = chunkWindow;
                        outputChunk->endTime = chunk.endTime - std::chrono::microseconds( (int) (1000000.0*(chunk.data.size() - i)/sampleRate));
                        //This ^ calculates the time the resized chunk ends given the time the chunk received by the trigger ends and how far through the received chunk we are
                        emit(std::move(outputChunk));
                        chunkWindow.clear();
                        chunkWindowSize = 0;
                        auto outputChunk = std::make_unique<resizedChunk>();
                    }
                }
                
            }
        });
        
        on<Trigger<resizedChunk>> ([this](const resizedChunk& resizedChunk) {
            // The last resized chunk of size HOP_SIZE (512) of sound data that were recorded are received and processed by aubio tempo track
            
            callCount++; //used to calculate numMilliseconds (probably redundant due to SoundChunk being able to keep track of time in terms of system clock)

            std::vector<int16_t> chunk = resizedChunk.resizedChunk;
            //chunk.insert( chunk.end(), resizedChunk.resizedChunk.begin(), resizedChunk.resizedChunk.end() );

            float numMilliseconds = (1000.0 * HOP_SIZE * callCount) /  sampleRate ;
            std::vector<int> results;


            for (int i = 0; i < HOP_SIZE; ++i)
            {
                for (int j = 0; j < channels; ++j)
                {
                    fvec_write_sample(tempo_input_data,  chunk.at(channels*i + j), j, i); //for channel k
                }
            }

            aubio_tempo(att, tempo_input_data, out);     
            for (int i =0; i < out->data[0][0]; ++i)
            {
                //std::cout << "Beats this call: " << out->data[0][0] << std::endl;
                beatCount += 1;
                std::cout << "Beat value " << beatCount << " : " << (out->data[0][i+1] + HOP_SIZE*(callCount-1))/(HOP_SIZE*callCount) * (numMilliseconds/1000) << "s \n";
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