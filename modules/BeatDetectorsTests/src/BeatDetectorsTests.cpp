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
#include <chrono>
#include <ctime>
#include "messages/Beat.h"
#include <vector>

namespace modules {



    BeatDetectorsTests::BeatDetectorsTests(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<messages::Beat>> ([this](const messages::Beat& beat) {
            
            std::time_t time = std::chrono::system_clock::to_time_t(beat.time);
            std::cout << "Beat found at: " << ctime(&time) << "Period: " << (0.0 + beat.period.count())/1000000000 << " seconds" << std::endl << std::endl;
        });

    }
    
    BeatDetectorsTests::~BeatDetectorsTests()
    {

    }

}