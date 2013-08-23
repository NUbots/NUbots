/*
 * This file is part of AudioInput.
 *
 * AudioInput is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * AudioInput is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with AudioInput.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "AudioInput.h"
#include "messages/SoundChunk.h"

// OpenAL
#include <AL/al.h>
#include <AL/alc.h>
#include <stdexcept>

// Errors
#include <system_error>
#include "utility/error/openal_error_category.h"

namespace modules {

    AudioInput::AudioInput(NUClear::PowerPlant* plant) : Reactor(plant) {
        
        const int SAMPLE_RATE = 44100;
        const int SAMPLE_SIZE = 1024;
        
        // Open a capture device
        ALCdevice* device = alcCaptureOpenDevice(nullptr, SAMPLE_RATE, AL_FORMAT_STEREO16, SAMPLE_SIZE);
        if (alGetError() != AL_NO_ERROR) {
            throw std::system_error(
                    alGetError(), 
                    utility::error::openal_error_category(), 
                    "There was an error while attempting to access the microphone");
        }
        
        // Start capturing
        alcCaptureStart(device);
        
        on<Trigger<Every<100, std::chrono::milliseconds>>>([this, device](const time_t&) {
            
            messages::SoundChunk* chunk = new messages::SoundChunk();
            
            int available = 0;
            alcGetIntegerv(device, ALC_CAPTURE_SAMPLES, sizeof(ALint), &available);
            
            chunk->data.resize(available);
            alcCaptureSamples(device, chunk->data.data(), available);
            
            emit(chunk);
        });
        
        on<Trigger<Shutdown>>([device] (const Shutdown&) {
            
            // Stop capturing and close the device
            alcCaptureStop(device);
            alcCloseDevice(device);
        });
    }
}
