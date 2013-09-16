/*
 * This file is part of BeatDetector.
 *
 * BeatDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeatDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeatDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "BeatDetector.h"
#include <aubio/aubio.h>
#include <chrono>
#include <ctime>
#include "messages/SoundChunk.h"
#include "messages/BeatLocations.h"


const int WINDOW_SIZE = 1600;
const int CHANNELS = 1;

namespace modules {
    BeatDetector::BeatDetector(NUClear::PowerPlant* plant) : Reactor(plant) {

        aubio_beattracking_t* tracker = new_aubio_beattracking(WINDOW_SIZE, CHANNELS);
        fvec_t* in = new_fvec(WINDOW_SIZE, CHANNELS);
        fvec_t* out = new_fvec(WINDOW_SIZE/4, CHANNELS);

        aubio_beattracking_do(tracker, in, out);

        on<Trigger<messages::SoundChunk>>([this, tracker, in, out](const messages::SoundChunk& chunk) {

            for(size_t i = 0; i < chunk.data.size(); ++i) {
                in->data[0][i] = chunk.data[i];
            }

            aubio_beattracking_do(tracker, in, out);

            std::cout << out->data[0][0] << std::endl;
        });
    }
}