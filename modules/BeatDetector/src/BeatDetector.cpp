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
#include "utility/idiom/pimpl_impl.h"

#include <aubio/aubio.h>
#include <chrono>
#include <ctime>
#include "messages/SoundChunk.h"
#include "messages/BeatLocations.h"


const int WINDOW_SIZE = 1600;
const int CHANNELS = 1;

namespace modules {
    class BeatDetector::impl {
        public:

    };

    BeatDetector::BeatDetector(NUClear::PowerPlant* plant) : Reactor(plant) {
        on<Trigger<messages::SoundChunk>>([this](const messages::SoundChunk& chunk) {
            log("Got chunk: ", chunk.data.size());
        });

        // Parameters are, onset detection type, buffer size, overlap size, channels
        /*aubio_tempo_t* tempo = new_aubio_tempo(aubio_onset_kl, WINDOW_SIZE, WINDOW_SIZE / 2, CHANNELS);

        fvec_t* in = new_fvec(WINDOW_SIZE, CHANNELS);
        fvec_t* out = new_fvec(2, CHANNELS);

        on<Trigger<messages::SoundChunk>>([this, tempo, in, out](const messages::SoundChunk& chunk) {

            for(size_t i = 0; i < chunk.data.size(); ++i) {
                in->data[0][i] = chunk.data[i];
            }

            aubio_tempo(tempo, in, out);

            std::cout << out->data[0][1] << std::endl;
        });*/
    }
}
