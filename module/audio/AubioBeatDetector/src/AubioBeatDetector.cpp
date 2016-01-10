/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "AubioBeatDetector.h"
#include "utility/idiom/pimpl_impl.h"

extern "C" {
    #include <aubio/aubio.h>
}

#include <chrono>
#include <ctime>
#include <vector>

#include "message/input/SoundChunk.h"
#include "message/audio/Beat.h"

namespace module {
    namespace audio {

        using message::input::SoundChunkSettings;
        using message::input::SoundChunk;

        const int WINDOW_SIZE = 1024;
        const int HOP_SIZE = 512; //number of frames to input to beat detection at one time

        class AubioBeatDetector::impl {
        public:
            //Chunk resizing variables
            std::vector<int16_t> chunkWindow;

            //Beat Detection Trigger Variables
            aubio_tempo_t* tempoTracker;
            fvec_t* outputData;
            fvec_t* inputData;

            // Settings
            size_t sampleRate;
            size_t channels;
            size_t chunkSize;
            size_t offset;
        };

        // Holds beat value without a period, used to create a Beat objects with a period by comparing the last 2 beatTime's
        struct BeatTime {
            NUClear::clock::time_point time;
        };

        AubioBeatDetector::AubioBeatDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<SoundChunkSettings>>().then([this](const SoundChunkSettings& settings) {

                // Store the settings of the sound chunks
                m->sampleRate = settings.sampleRate;
                m->channels = settings.channels;
                m->chunkSize = settings.chunkSize;

                // Build our audio tempo tracker  can set to (aubio_onset_kl or aubio_onset_complex onset tracking)
                m->tempoTracker = new_aubio_tempo(aubio_onset_kl, WINDOW_SIZE, HOP_SIZE, m->channels);
                m->outputData = new_fvec(HOP_SIZE, m->channels);
                m->inputData = new_fvec(HOP_SIZE, m->channels);


            });

            on<Trigger<SoundChunk>>().then([this](const SoundChunk& chunk) {

                for (size_t i = 0; i < m->chunkSize; ++i) {

                    // Write to our vector
                    size_t index = (i + m->offset) % HOP_SIZE;
                    fvec_write_sample(m->inputData, chunk.data[i * m->channels], 0, index);

                    // If we are done filling this hop chunk
                    if(index == HOP_SIZE - 1) {

                        aubio_tempo(m->tempoTracker, m->inputData, m->outputData);
                        for (int i = 1; i <= m->outputData->data[0][0]; ++i) {

                            auto beatTime = std::make_unique<BeatTime>();

                            // Work out how many samples from the end we are in total
                            const size_t position = ((i - HOP_SIZE) + (m->outputData->data[0][i]));

                            // Start at our end time and go back
                            beatTime->time = chunk.endTime - NUClear::clock::duration(int(NUClear::clock::period::den * (double(position) / m->sampleRate)));

                            emit(std::move(beatTime));
                        }
                    }
                }

                m->offset = (m->chunkSize + m->offset) % HOP_SIZE;
            });

           on<Last<2, Trigger<BeatTime>>>().then([this](const std::list<std::shared_ptr<const BeatTime>>& lastTwoBeats) {

               if(lastTwoBeats.size() == 2) {
                    auto beat = std::make_unique<message::audio::Beat>();
                    beat->time = lastTwoBeats[0]->time; //apparently the latest one is 0
                    beat->period = lastTwoBeats[0]->time - lastTwoBeats[1]->time;

                    emit(std::move(beat));
               }
           });

           on<Shutdown>().then([this] {
              del_aubio_tempo(m->tempoTracker);
              del_fvec(m->inputData);
              del_fvec(m->outputData);
           });
        }
    }  // audio
}  // modules