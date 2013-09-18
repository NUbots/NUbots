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
#include "messages/DarwinSensors.h"
#include "messages/BeatLocations.h"
#include "utility/math/angle.h"

namespace modules {

    const int NUM_BEAT_SAMPLES = 1000;

    struct Buckets {
        std::vector<double> buckets;
    };
    struct DiffBuckets {
        std::vector<double> buckets;
    };
    struct Beat {
        double quality;
        double phase;
        double frequency;
    };
    struct FilteredBeat {
        double phase;
        double frequency;
    };

    class BeatDetector::impl {
    public:
        size_t channels;
        size_t sampleRate;
        size_t chunkSize;
        volatile bool allowBeat;

        fftw_plan plan;
        std::unique_ptr<double[]> input;
        std::unique_ptr<std::complex<double>[]> output;

        fftw_plan beatPlan;
        std::unique_ptr<double[]> beatInput;
        std::unique_ptr<std::complex<double>[]> beatOutput;

        double frequency(const size_t& index, const size_t& sampleRate, const size_t& fftSize);

        std::vector<double> getBuckets(const messages::SoundChunk& chunk);
        Beat findBeat(const std::vector<DiffBuckets>& diffbuckets);

    };

    const size_t BUCKET_BOUNDRY[] = {
            200,
            400,
            800,
            1600,
            3200
    };

    double BeatDetector::impl::frequency(const size_t& index, const size_t& sampleRate, const size_t& fftSize) {
        return double(index) * double(sampleRate) / double(fftSize);
    }

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
            if(frequency(i, sampleRate, chunkSize) > BUCKET_BOUNDRY[bucket]) {
                if(++bucket >= sizeof(BUCKET_BOUNDRY) / sizeof(int)) {
                    break;
                }
            }

            values[bucket] += abs(output[i]);
        }

        return values;
    }

    Beat BeatDetector::impl::findBeat(const std::vector<DiffBuckets>& buckets) {

        std::vector<std::complex<double>> values(NUM_BEAT_SAMPLES / 2, 0);

        for (size_t b = 0; b < buckets[0].buckets.size(); ++b) {
            for (size_t i = 0; i < NUM_BEAT_SAMPLES; ++i) {
                beatInput[i] = buckets[i].buckets[b];
            }

            fftw_execute(beatPlan);

            for(int i = 0; i < NUM_BEAT_SAMPLES / 2; ++i) {
                values[i] += beatOutput[i];
            }
        }

        // We start at 60bpm
        size_t start = (60 * NUM_BEAT_SAMPLES) / 6000;
        // and end at 200bpm
        size_t end = (200 * NUM_BEAT_SAMPLES) / 6000;
        double average = 0;

        // Get our average
        for(size_t i = start; i < end + 1; ++i) {
            average += abs(values[i]);
        }

        // Normalize our average
        average /= (end - start) + 1;

        // Find the candidates
        std::vector<Beat> candidates;
        for(size_t i = start; i < end + 1; ++i) {
            if(abs(values[i]) > average) {
                Beat data;

                // If we want to move to our left
                if(abs(values[i - 1]) > abs(values[i + 1])) {
                    data.phase = std::arg(values[i]);
                    data.frequency = (frequency(i - 1, 100, NUM_BEAT_SAMPLES) * abs(values[i - 1]) + frequency(i, 100, NUM_BEAT_SAMPLES) * abs(values[i])) / (abs(values[i - 1]) + abs(values[i]));
                    data.quality = 0;
                }
                // Otherwise moving to the right
                else {
                    data.phase = std::arg(values[i]);
                    data.frequency = (frequency(i + 1, 100, NUM_BEAT_SAMPLES) * abs(values[i + 1]) + frequency(i, 100, NUM_BEAT_SAMPLES) * abs(values[i])) / (abs(values[i + 1]) + abs(values[i]));
                    data.quality = 0;
                }

                candidates.push_back(std::move(data));
            }
        }

        // Now we need to build up the quality of each candidate by searching multiples of its beat
        for(auto& candidate : candidates) {
            for(int i = 0; i < 5; ++i) {
                double checkFrequency = candidate.frequency * (1 << i);

                // Work out the two indicies and the relative influence
                size_t indexA = floor((checkFrequency * NUM_BEAT_SAMPLES) / 100);
                size_t indexB = ceil((checkFrequency * NUM_BEAT_SAMPLES) / 100);
                double influenceA = abs(checkFrequency - frequency(indexA, NUM_BEAT_SAMPLES, 100));
                double influenceB = abs(checkFrequency - frequency(indexB, NUM_BEAT_SAMPLES, 100));
                double scale = 1 / (influenceA * influenceB);
                influenceA *= scale;
                influenceB *= scale;

                candidate.quality += abs(values[indexA]) * influenceA;
                candidate.quality += abs(values[indexB]) * influenceB;
            }
        }

        std::sort(std::begin(candidates), std::end(candidates), [](const Beat& a, const Beat& b) {
            return a.quality > b.quality;
        });

        // If we have candidates
        if(!candidates.empty()) {
            //std::cout << "Frequency: " << candidates.front().frequency * 60 << " Phase: " << candidates.front().phase << " Quality: " << candidates.front().quality << std::endl;

            return candidates.front();
        }
        else {
            return Beat();
        }
    }

    BeatDetector::BeatDetector(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<messages::SoundChunkSettings>>([this](const messages::SoundChunkSettings& settings) {

            // Store the settings
            m->sampleRate = settings.sampleRate;
            m->channels = settings.channels;
            m->chunkSize = settings.chunkSize;
            m->allowBeat = true;

            // Allocate our memory for input and output
            m->input = std::unique_ptr<double[]>(new double[m->chunkSize]);
            m->output = std::unique_ptr<std::complex<double>[]>(new std::complex<double>[(m->chunkSize/2) + 1]);

            // Build our plan for FFT
            m->plan = fftw_plan_dft_r2c_1d(m->chunkSize, m->input.get(), reinterpret_cast<fftw_complex*>(m->output.get()), 0);

            // Allocate our memory for our beat fft
            m->beatInput = std::unique_ptr<double[]>(new double[NUM_BEAT_SAMPLES]);
            m->beatOutput = std::unique_ptr<std::complex<double>[]>(new std::complex<double>[(NUM_BEAT_SAMPLES / 2) + 1]);

            // Build our plan for Beat FFT
            m->beatPlan = fftw_plan_dft_r2c_1d(NUM_BEAT_SAMPLES, m->beatInput.get(), reinterpret_cast<fftw_complex*>(m->beatOutput.get()), 0);
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

        on<Trigger<Last<NUM_BEAT_SAMPLES, DiffBuckets>>>([this](const std::vector<std::shared_ptr<const DiffBuckets>>& input) {

            std::vector<DiffBuckets> buckets;
            buckets.reserve(NUM_BEAT_SAMPLES);

            for(size_t i = 0; i < input.size(); ++i) {
                buckets.push_back(*input[i]);
            }
            for(size_t i = input.size(); i < NUM_BEAT_SAMPLES; ++i) {
                DiffBuckets empty;
                empty.buckets.resize(buckets.front().buckets.size(), 0);
                buckets.push_back(empty);
            }

            auto beat = std::make_unique<Beat>();
            *beat = m->findBeat(buckets);
            emit(std::move(beat));
        });

        on<Trigger<Last<100, Beat>>>([this](const std::vector<std::shared_ptr<const Beat>>& input) {

            std::vector<Beat> beats;
            std::vector<size_t> indexes;

            for(size_t i = 0; i < input.size(); ++i) {
                beats.push_back(*input[i]);
                indexes.push_back(i);
            }

            std::sort(std::begin(indexes), std::end(indexes), [beats](const size_t& a, const size_t& b) {
                return beats[a].frequency < beats[b].frequency;
            });

            size_t index = indexes[indexes.size() / 2];
            Beat beat = beats[index];

            double phaseOffset = (indexes.size() - index) * ((2 * M_PI) / (100.0 / beat.frequency));
            double phase = utility::math::angle::normalizeAngle(beat.phase + phaseOffset);

            auto filtered = std::make_unique<FilteredBeat>();
            filtered->frequency = beat.frequency;
            filtered->phase = phase;

            emit(std::move(filtered));
        });

        on<Trigger<Last<2, FilteredBeat>>>([this](const std::vector<std::shared_ptr<const FilteredBeat>>& input) {
            if(input.size() == 2) {
                if(m->allowBeat
                   && input[1]->phase > 0
                   && input[1]->phase < M_PI_2
                   && input[0]->phase < 0
                   && input[0]->phase > -M_PI_2) {

                    auto eyes = std::make_unique<messages::DarwinSensors::EyeLED>();
                    eyes->r = 0xFF * (double(rand()) / double(RAND_MAX));
                    eyes->g = 0xFF * (double(rand()) / double(RAND_MAX));
                    eyes->b = 0xFF * (double(rand()) / double(RAND_MAX));
                    emit(std::move(eyes));

                    m->allowBeat = false;
                }
                else if(input[0]->phase > M_PI_2 && input[1]->phase < -M_PI_2) {
                    m->allowBeat = true;
                }
            }
        });
    }
}
