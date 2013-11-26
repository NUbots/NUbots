/*
 * This file is part of BeatDetectorsTests.
 *
 * AudioInput is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * BeatDetectorsTests is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with BeatDetectorsTests.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Joshua Kearns <joshau-k@hotmail.com>
 */

#include "BeatDetectorsTests.h"
#include "utility/idiom/pimpl_impl.h"

#include "messages/Beat.h"
#include "messages/SoundChunk.h"

#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <fstream>

namespace modules {

    class BeatDetectorsTests::impl {
    public:
        //Records start time of Audio
        NUClear::clock::time_point startTime;
        std::string fileName;
        std::ofstream myfile;
        bool soundFileStarted = false;
    };
    
    

    BeatDetectorsTests::BeatDetectorsTests(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        
        
        on<Trigger<messages::SoundFileStart>, Options<Single>> ([this](const messages::SoundFileStart& soundFileStart) {
            m->myfile.close();
            
            m->startTime = soundFileStart.time;
            m->fileName = soundFileStart.fileName;
            
            m->myfile.open (std::string(m->fileName) + std::string(".txt"), std::ios::out);
            m->myfile.exceptions(std::ofstream::badbit | std::ofstream::failbit);
                
            m->soundFileStarted = true;
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            
        });
         
        /*
        on<Trigger<messages::Beat>> ([this](const messages::Beat& beat) {

            //while (m->soundFileStarted == false)
            //{}
            if (m->soundFileStarted == true)
            {
                NUClear::clock::duration relativeTime = beat.time - m->startTime;
                //std::time_t time = std::chrono::system_clock::to_time_t(beat.time);
                //std::time_t relativeTime = std::chrono::system_clock::to_time_t(relativeTimeDuration);

                float secs, millis;
                millis = (std::chrono::duration_cast<std::chrono::milliseconds>(relativeTime)).count();
                secs = millis /1000;

                //std::cout << "Beat found at: " << secs
                //        << "Period: " << 60 / (double(beat.period.count()) / double(NUClear::clock::period::den)) << "bpm"
                //        << std::endl << std::endl;


                m->myfile << secs << std::endl;
                m->myfile.flush();
            }
        });
        
       on<Trigger<Shutdown>>([this](const Shutdown&) {
            m->myfile.close();
       });*/
        on<Trigger<messages::Beat>> ([this](const messages::Beat& beat) {
            //std::cout << "beat received" << std::endl;
            //while (m->soundFileStarted == false)
            //{}
            if (m->soundFileStarted == true)
            {
                NUClear::clock::duration relativeTime = beat.time - m->startTime;
                //std::time_t time = std::chrono::system_clock::to_time_t(beat.time);
                //std::time_t relativeTime = std::chrono::system_clock::to_time_t(relativeTimeDuration);

                float secs, millis;
                millis = (std::chrono::duration_cast<std::chrono::milliseconds>(relativeTime)).count();
                secs = millis /1000;

                //std::cout << "Beat found at: " << secs
                //        << "Period: " << 60 / (double(beat.period.count()) / double(NUClear::clock::period::den)) << "bpm"
                //        << std::endl << std::endl;


                std::cout << secs << std::endl;
                //m->myfile << secs << std::endl;
                //m->myfile.flush();
                
            }
        });
    }
    


}