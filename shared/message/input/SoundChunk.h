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

#ifndef MESSAGE_INPUT_SOUNDCHUNK_H
#define MESSAGE_INPUT_SOUNDCHUNK_H

#include <nuclear>
#include <vector>
#include <cstdint>

namespace message {
    namespace input {

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct SoundChunkSettings {
            /// The number of samples that are taken each second for the sound chunks
            size_t sampleRate;
            /// The number of channels that the sound chunks will have
            size_t channels;
            /// The number of frames (a frame is a single sample for all channels) that each emitted chunk will have
            size_t chunkSize;
        };


        struct SoundFileStart {

            std::string fileName;
            NUClear::clock::time_point time;
        };

        /**
         * TODO document
         *
         * @author Jake Woods
         */
        struct SoundChunk {

            NUClear::clock::time_point endTime;
            std::vector<int16_t> data;
        };

    }  // input
}  // message

#endif  // MESSAGE_INPUT_SOUNDCHUNK_H

