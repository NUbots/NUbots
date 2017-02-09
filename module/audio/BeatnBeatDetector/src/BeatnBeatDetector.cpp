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

#include "BeatDetector.h"
#include "utility/idiom/pimpl_impl.h"
#include <complex>

extern "C" {
    #include <fftw3.h>
}

#include "message/SoundChunk.h"
#include "message/DarwinSensors.h"
#include "message/Beat.h"
#include "utility/math/angle.h"

namespace module {
    namespace audio {

        /// This is the number of samples that we will consider while caclculating the beat
        const int NUM_BEAT_SAMPLES = 1000;

        /// Holds a set of buckets for us to use (fft slices)
        struct Buckets {
            std::vector<double> buckets;
        };

        /// Holds the derivitive of a stream of incoming buckets
        struct DiffBuckets {
            std::vector<double> buckets;
        };

        // Contains a beat that was detected in our sliding window
        struct Beat {
            double quality;
            double phase;
            double frequency;
        };

        /// Contains a beat after it has been filtered using the median filter
        struct FilteredBeat {
            double phase;
            double frequency;
        };

        class BeatDetector::impl {
        public:
            /// The number of channels in our input data (we only consider the first channel)
            size_t channels;
            /// The sample rate of our incoming data
            size_t sampleRate;
            /// How large each chunk of data that comes in will be (in frames not shorts)
            size_t chunkSize;
            /// If we will allow the emission of another beat
            volatile bool allowBeat;

            /// The fft plan that will be used for our audio input
            fftw_plan plan;
            /// The input for our audio fft
            std::unique_ptr<double[]> input;
            /// The output for our audio fft
            std::unique_ptr<std::complex<double>[]> output;

            /// The fft plan that will be used for our beats
            fftw_plan beatPlan;
            /// The input for our beat fft
            std::unique_ptr<double[]> beatInput;
            /// The output for our beat fft
            std::unique_ptr<std::complex<double>[]> beatOutput;

            /**
             * Calculates the central frequency for this input bucket.
             *
             * @param index         the fft array index
             * @param sampleRate    the sample rate of the incoming data to the fft
             * @param fftSize       the number of elements in the fft output (full array size, even
             *                      if it is halved by a real input)
             *
             * @return the central frequency for the fft bucket of the passed index
             */
            double frequency(const size_t& index, const size_t& sampleRate, const size_t& fftSize);

            /**
             * Gets the fft buckets (split into divisions of BUCKET_BOUNDRY) for the given sound chunk
             *
             * @param chunk the sound chunk to perform the fft on
             *
             * @return the summation of the absolute value of frequency buckets in the fft between the requested frequencies
             */
            std::vector<double> getBuckets(const message::SoundChunk& chunk);

            /**
             * Finds the most likely beat frequency given an array of differentiated input
             *
             * @param diffBuckets the stream of differentiated frequency buckets of the input
             *
             * @return A beat object of the most likely beat from the input
             */
            Beat findBeat(const std::vector<DiffBuckets>& diffbuckets);

        };

        /// The boundaries of each bucket
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

        std::vector<double> BeatDetector::impl::getBuckets(const message::SoundChunk& chunk) {

            // Copy our sound data from the first channel into the input buffer (we only care about the first channel)
            for (size_t i = 0; i < chunkSize; ++i) {
                input[i] = double(chunk.data[i * channels]) / double(std::numeric_limits<short>::max());
            }

            // Execute our FFT
            fftw_execute(plan);

            std::vector<double> values(sizeof(BUCKET_BOUNDRY) / sizeof(int), 0);

            // Loop through each bucket summing the values in for our ranges
            size_t bucket = 0;
            for(size_t i = 0; i < chunkSize/2; ++i) {
                if(frequency(i, sampleRate, chunkSize) > BUCKET_BOUNDRY[bucket]) {
                    if(++bucket >= sizeof(BUCKET_BOUNDRY) / sizeof(int)) {
                        break;
                    }
                }

                // Add in our absolute value (magnitude) of the imaginary number
                values[bucket] += abs(output[i]);
            }

            return values;
        }

        Beat BeatDetector::impl::findBeat(const std::vector<DiffBuckets>& buckets) {

            std::vector<std::complex<double>> values(NUM_BEAT_SAMPLES / 2, 0);

            // Calculate the FFT sample rate that we are using
            size_t fftSampleRate = sampleRate / chunkSize;

            // Perform the FFT on each bucket and sum their outputs
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
                        data.frequency = (frequency(i - 1, fftSampleRate, NUM_BEAT_SAMPLES) * abs(values[i - 1]) + frequency(i, fftSampleRate, NUM_BEAT_SAMPLES) * abs(values[i])) / (abs(values[i - 1]) + abs(values[i]));
                        data.quality = 0;
                    }
                    // Otherwise moving to the right
                    else {
                        data.phase = std::arg(values[i]);
                        data.frequency = (frequency(i + 1, fftSampleRate, NUM_BEAT_SAMPLES) * abs(values[i + 1]) + frequency(i, fftSampleRate, NUM_BEAT_SAMPLES) * abs(values[i])) / (abs(values[i + 1]) + abs(values[i]));
                        data.quality = 0;
                    }

                    candidates.push_back(std::move(data));
                }
            }

            // Now we need to build up the quality of each candidate by searching two multiples above and two below the beat
            for(auto& candidate : candidates) {
                for(int i = 0; i < 5; ++i) {
                    double checkFrequency = candidate.frequency * pow(2, i - 2);

                    // Work out the two indicies and the relative influence
                    size_t indexA = floor((checkFrequency * NUM_BEAT_SAMPLES) / fftSampleRate);
                    size_t indexB = ceil((checkFrequency * NUM_BEAT_SAMPLES) / fftSampleRate);

                    // We work out how different from our target frequency the two indexes are
                    double influenceA = abs(checkFrequency - frequency(indexA, fftSampleRate, NUM_BEAT_SAMPLES));
                    double influenceB = abs(checkFrequency - frequency(indexB, fftSampleRate, NUM_BEAT_SAMPLES));

                    // Now we want to work out how much of each bucket makes up this frequency, this means that
                    // the sum of the two scaling factors should be 1, we swap this so b=a and a=b as a smaller
                    // difference in frequency should actually result in a larger ratio
                    double scaleA = influenceB / (influenceA + influenceB);
                    double scaleB = influenceA / (influenceA + influenceB);

                    // Multiply in the values for this skewed by the frequency
                    candidate.quality += abs(values[indexA]) * scaleA;
                    candidate.quality += abs(values[indexB]) * scaleB;
                }
            }

            // Find the element with the highest quality
            if(!candidates.empty()) {
                return *(std::max_element(std::begin(candidates), std::end(candidates), [](const Beat& a, const Beat& b) {
                    return a.quality > b.quality;
                }));
            }
            // Return an empty beat object (this will happen when our average is 0 (because all elements are 0))
            else {
                return Beat();
            }
        }

        BeatDetector::BeatDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<message::SoundChunkSettings>>().then([this](const message::SoundChunkSettings& settings) {

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

            // This triggers on every sound chunk we get
            on<Trigger<message::SoundChunk>>().then([this](const message::SoundChunk& chunk) {
                // Split the sound into frequency buckets and emit
                auto buckets = std::make_unique<Buckets>();
                buckets->buckets = m->getBuckets(chunk);
                emit(std::move(buckets));
            });

            // This triggers on every 2 buckets we make, it does the derivative of the value
            on<Last<2, Trigger<Buckets>>>().then([this](const std::list<std::shared_ptr<const Buckets>>& buckets) {

                // If we have at least two buckets (we can't do derivative on 1)
                if(buckets.size() > 1) {
                    // Make a bucket the correct size
                    auto dbucket = std::make_unique<DiffBuckets>();
                    dbucket->buckets.resize(buckets[0]->buckets.size());

                    // Differentiate the buckets
                    for(int i = 0; i < 5; ++i) {
                        dbucket->buckets[i] = buckets[1]->buckets[i] - buckets[0]->buckets[i];
                        dbucket->buckets[i] = dbucket->buckets[i] < 0 ? 0 : dbucket->buckets[i];
                    }

                    emit(std::move(dbucket));
                }
            });

            // This takes in the last 10 seconds (assuming 100 chunks per second) of differentiated buckets and finds the
            // most likely beat from it
            on<Last<NUM_BEAT_SAMPLES, Trigger<DiffBuckets>>>().then([this](const std::vector<std::shared_ptr<const DiffBuckets>>& input) {

                // Allocate an array of buckets
                std::vector<DiffBuckets> buckets;

                // Make it the correct size
                buckets.reserve(NUM_BEAT_SAMPLES);

                // Put all the data we actually have into the array (will initially be 0)
                for(size_t i = 0; i < input.size(); ++i) {
                    buckets.push_back(*input[i]);
                }
                // Fill the remainder of the array with 0s
                for(size_t i = input.size(); i < NUM_BEAT_SAMPLES; ++i) {
                    DiffBuckets empty;
                    empty.buckets.resize(buckets.front().buckets.size(), 0);
                    buckets.push_back(empty);
                }

                // Find and emit the beat
                auto beat = std::make_unique<Beat>();
                *beat = m->findBeat(buckets);
                emit(std::move(beat));
            });

            // This takes the last 100 estimated beat locations and and gets the median. This will get rid of most of
            // the random outlier beats that are detected
            on<Last<100, Trigger<Beat>>>().then([this](const std::vector<std::shared_ptr<const Beat>>& input) {

                // Make vectors for our beats and the indexes to the beats (so we can calculate our phase offset)
                std::vector<Beat> beats;
                std::vector<size_t> indexes;

                // Fill the beats and indexes
                for(size_t i = 0; i < input.size(); ++i) {
                    beats.push_back(*input[i]);
                    indexes.push_back(i);
                }

                // Sort the indexes array based on the frequency of the beats
                std::sort(std::begin(indexes), std::end(indexes), [beats](const size_t& a, const size_t& b) {
                    return beats[a].frequency < beats[b].frequency;
                });

                // Get our median index and the beat for that index
                size_t index = indexes[indexes.size() / 2];
                Beat beat = beats[index];

                // Calculate our phase offset between when this median beat was and now
                double phaseOffset = (indexes.size() - index) * ((2 * M_PI) / ((m->sampleRate / m->chunkSize) / beat.frequency));
                double phase = utility::math::angle::normalizeAngle(beat.phase + phaseOffset);

                // Make and emit our filtered beat with the frequency and calculated phase offset
                auto filtered = std::make_unique<FilteredBeat>();
                filtered->frequency = beat.frequency;
                filtered->phase = phase;
                emit(std::move(filtered));
            });

            // This takes the last two filtered beats and emits a beat message every full phase revolution)
            on<Last<2, Trigger<FilteredBeat>>>().then([this](const std::vector<std::shared_ptr<const FilteredBeat>>& input) {

                // Make sure we have 2 elements
                if(input.size() == 2) {

                    // If we are allowed to emit another beat and we have two beat values that cross between -tive and +tive
                    if(m->allowBeat
                       && input[1]->phase > 0
                       && input[1]->phase < M_PI_2
                       && input[0]->phase < 0
                       && input[0]->phase > -M_PI_2) {

                        // Make our beat object and emit it
                        auto beat = std::make_unique<message::Beat>();
                        beat->time = NUClear::clock::now();
                        beat->period = NUClear::clock::duration(static_cast<long>((1 / input[0]->frequency) * NUClear::clock::period::den));
                        emit(std::move(beat));

                        // We cannot emit another beat until we have crossed into the other half of the phase (to remove noise)
                        m->allowBeat = false;
                    }
                    // If we cross into the other half of the phase, then we reset that we can emit a beat
                    else if(!m->allowBeat && input[0]->phase > M_PI_2 && input[1]->phase < -M_PI_2) {
                        m->allowBeat = true;
                    }
                }
            });
        }
    }  // audio
}  // modules
