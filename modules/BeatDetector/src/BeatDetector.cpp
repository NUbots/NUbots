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
#include <complex>

extern "C" {
    #include <fftw3.h>
}

#include "messages/SoundChunk.h"
#include "messages/BeatLocations.h"

namespace modules {

    struct Buckets {
        std::vector<double> buckets;
    };
    struct DiffBuckets {
        std::vector<double> buckets;
    };

    class BeatDetector::impl {
    public:
        size_t channels;
        size_t sampleRate;
        size_t chunkSize;

        fftw_plan plan;
        std::unique_ptr<double[]> input;
        std::unique_ptr<std::complex<double>[]> output;


        fftw_plan beatPlan;
        std::unique_ptr<double[]> beatInput;
        std::unique_ptr<std::complex<double>[]> beatOutput;

        std::vector<double> getBuckets(const messages::SoundChunk& chunk);
        double findBeat(const std::vector<std::shared_ptr<const DiffBuckets>>& diffbuckets);

    };

    const size_t BUCKET_BOUNDRY[] = {
            200,
            400,
            800,
            1600,
            3200
    };

    std::vector<double> BeatDetector::impl::getBuckets(const messages::SoundChunk& chunk) {

        // Copy our sound data from the first channel into the input buffer (we only care about the first channel)
        for (size_t i = 0; i < chunkSize; ++i) {
            input[i] = double(chunk.data[i * channels]) / double(std::numeric_limits<short>::max());
        }

        // Execute our FFT
        fftw_execute(plan);

        std::vector<double> values(sizeof(BUCKET_BOUNDRY) / sizeof(int), 0);

        size_t bucket = 0;
        for(size_t i = 0; i < chunkSize/2; ++i) {
            if((i * sampleRate / chunkSize) > BUCKET_BOUNDRY[bucket]) {
                if(++bucket >= sizeof(BUCKET_BOUNDRY) / sizeof(int)) {
                    break;
                }
            }

            values[bucket] += abs(output[i]);
        }

        return values;
    }

    double BeatDetector::impl::findBeat(const std::vector<std::shared_ptr<const DiffBuckets>>& buckets) {

        std::vector<std::complex<double>> values(500, 0);

        for (size_t b = 0; b < buckets[0]->buckets.size(); ++b) {
            for (size_t i = 0; i < 1000; ++i) {
                beatInput[i] = buckets[i]->buckets[b];
            }

            fftw_execute(beatPlan);

            for(int i = 0; i < 500; ++i) {
                values[i] += beatOutput[i];
            }
        }

        // Now we look though the frequencies from 50hz to 200hz (bpm of music) and find our average

        // Now we look for any of those values that are significantly above average, these are our candidates

        // First we try to get a better value for our candidates, see which bucket around it is largest (left or right)
        // and normalize based off this (to get closer to the "real" value)

        // Now we go skipping through the array trying to find the best multiples see how big freq*2, freq*4 etc is for our candidate

        // The lowest candidate with good multiples is our frequency

        for(int i = 0; i < 500; ++i) {
            std::cout << abs(values[i]) << " ";
        }
        std::cout << std::endl;

        return 0;
    }

    BeatDetector::BeatDetector(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<messages::SoundChunkSettings>>([this](const messages::SoundChunkSettings& settings) {

            // Store the settings
            m->sampleRate = settings.sampleRate;
            m->channels = settings.channels;
            m->chunkSize = settings.chunkSize;

            // Allocate our memory for input and output
            m->input = std::unique_ptr<double[]>(new double[m->chunkSize]);
            m->output = std::unique_ptr<std::complex<double>[]>(new std::complex<double>[(m->chunkSize/2) + 1]);

            // Build our plan for FFT
            m->plan = fftw_plan_dft_r2c_1d(m->chunkSize, m->input.get(), reinterpret_cast<fftw_complex*>(m->output.get()), 0);

            // Allocate our memory for our beat fft
            m->beatInput = std::unique_ptr<double[]>(new double[1000]);
            m->beatOutput = std::unique_ptr<std::complex<double>[]>(new std::complex<double>[501]);

            // Build our plan for Beat FFT
            m->beatPlan = fftw_plan_dft_r2c_1d(1000, m->beatInput.get(), reinterpret_cast<fftw_complex*>(m->beatOutput.get()), 0);
        });


        on<Trigger<messages::SoundChunk>>([this](const messages::SoundChunk& chunk) {

            auto buckets = std::make_unique<Buckets>();
            buckets->buckets = m->getBuckets(chunk);

            emit(std::move(buckets));
        });

        on<Trigger<Last<2, Buckets>>>([this](const std::vector<std::shared_ptr<const Buckets>>& buckets) {

            if(buckets.size() > 1) {

                auto dbucket = std::make_unique<DiffBuckets>();
                dbucket->buckets.resize(5);

                for(int i = 0; i < 5; ++i) {
                    dbucket->buckets[i] = buckets[1]->buckets[i] - buckets[0]->buckets[i];
                    dbucket->buckets[i] = dbucket->buckets[i] < 0 ? 0 : dbucket->buckets[i];
                }

                emit(std::move(dbucket));
            }
        });

        on<Trigger<Last<1000, DiffBuckets>>>([this](const std::vector<std::shared_ptr<const DiffBuckets>>& buckets) {

            if(buckets.size() == 1000) {
                auto data = m->findBeat(buckets);
            }
        });
    }
}
