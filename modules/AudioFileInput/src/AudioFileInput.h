/*
 * This file is part of AudioFileInput.
 *
 * AudioFileInput is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
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
 * Copyright 2013 Joshua Kearns <joshau-k@hotmail.com>
 */

#ifndef MODULES_AUDIOFILEINPUT_H
#define MODULES_AUDIOFILEINPUT_H

#include <NUClear.h>
//#include "RtAudio.h"

namespace modules {

    class AudioFileInput : public NUClear::Reactor {
        public:
            explicit AudioFileInput(NUClear::PowerPlant* plant);
        //private:
            // TODO: Consider replacing this with the PIMPL idiom to remove
            // the RtAudio header dependency.
            //RtAudio audioContext;
            //RtAudio::StreamParameters inputStreamParameters;
    };
}
#endif

