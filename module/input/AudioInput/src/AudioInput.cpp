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

#include <algorithm>

#include "AudioInput.h"
#include "utility/idiom/pimpl_impl.h"

#include "message/SoundChunk.h"

#include "RtAudio.h"

namespace module {
    namespace input {

        const int CHUNKS_PER_SECOND = 100;

        namespace {
            // Holds the various user data we're passing to our audio callback.
            struct UserData {
                UserData(NUClear::PowerPlant* powerPlant,
                        RtAudio::StreamParameters& inputStreamParameters,
                        unsigned int sampleRate) :
                    powerPlant(powerPlant),
                    inputStreamParameters(inputStreamParameters),
                    sampleRate(sampleRate) {}

                NUClear::PowerPlant* powerPlant;
                RtAudio::StreamParameters& inputStreamParameters;
                unsigned int sampleRate;
            };

            int audioCallback(
                    void* outputBuffer,
                    void* inputBuffer,
                    unsigned int nBufferFrames,
                    double streamTime,
                    RtAudioStreamStatus status,
                    void* userData ) {

                if(status) {
                    // If status is false then we've detected a stream overflow.
                    // TODO: Add some error handling here
                }

                int16_t* data = (int16_t*)inputBuffer;

                // This is a horrible hack to get some C++-space data
                // into this function.
                UserData* udata = static_cast<UserData*>(userData);

                auto chunk = std::make_unique<message::SoundChunk>();

                // Sound is split up into left and right channels which are 16 bytes
                // (for the darwin) each. This means a single frame actually compromises
                // 32 bits and we need to allocate data appropriately.
                const int CHANNELS = udata->inputStreamParameters.nChannels;

                chunk->data.resize(nBufferFrames * CHANNELS);
                chunk->data.assign(data, data + (nBufferFrames * CHANNELS));
                chunk->endTime = NUClear::clock::now();
                udata->powerPlant->emit(std::move(chunk));

                return 0;
            }
        }

        // Implement our impl class.
        class AudioInput::impl {
            public:
                RtAudio audioContext;
                RtAudio::StreamParameters inputStreamParameters;
                std::unique_ptr<UserData> userData;
        };

        AudioInput::AudioInput(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            auto& audioContext = m->audioContext;
            auto& inputStreamParameters = m->inputStreamParameters;

            // We need to set up the audio context.
            if(audioContext.getDeviceCount() < 1) {
                NUClear::log<NUClear::DEBUG>("No audio devices found");
                return;
            }

            // We're going to look for our device by name. In the future this should
            // pull from the config system and use on<Startup>().
            int deviceNumber = audioContext.getDefaultInputDevice();
            RtAudio::DeviceInfo info = audioContext.getDeviceInfo(deviceNumber);
            for(unsigned int i = 0; i < audioContext.getDeviceCount(); ++i) {
                info = audioContext.getDeviceInfo(i);
                if(info.isDefaultInput) {
                    deviceNumber = i;
                    break;
                }
            }

            // Use the DeviceInfo we retrieved in the previous section to determine our
            // sampling, channels and other things.
            inputStreamParameters.deviceId = deviceNumber;
            inputStreamParameters.nChannels = info.inputChannels;
            inputStreamParameters.firstChannel = 0;

            unsigned int sampleRate = *(std::max_element(info.sampleRates.begin(), info.sampleRates.end()));
            unsigned int bufferFrames = sampleRate / CHUNKS_PER_SECOND; // 256 sample frames.

            m->userData = std::make_unique<UserData>(powerPlant, inputStreamParameters, sampleRate);

            auto settings = std::make_unique<message::SoundChunkSettings>();
            settings->sampleRate = sampleRate;
            settings->channels = info.inputChannels;
            settings->chunkSize = sampleRate / CHUNKS_PER_SECOND;

            emit<Scope::INITIALIZE>(std::move(settings));

            try {
                /**
                 * We're assuming that the input will be a signed 16 bit integer but this may
                 * not be supported on all input devices. Auto-detection or configuration would
                 * be useful here.
                 */
                audioContext.openStream(
                        NULL,
                        &inputStreamParameters,
                        RTAUDIO_SINT16,
                        sampleRate,
                        &bufferFrames,
                        &audioCallback,
                        m->userData.get()
                );

                audioContext.startStream();
            } catch( RtError& e) {
                e.printMessage();
                return;
            }
        }

    }  // input
}  // modules
