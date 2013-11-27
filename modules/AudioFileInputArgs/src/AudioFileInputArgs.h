/** 
 * This file is part of AudioFileInputArgs.
 *
 * AudioFileInputArgs is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
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

#ifndef MODULES_AUDIOFILEINPUTARGS_H
#define MODULES_AUDIOFILEINPUTARGS_H

#include <nuclear>
#include "utility/idiom/pimpl.h"

namespace modules {

    class AudioFileInputArgs : public NUClear::Reactor {
        private:
            class impl;
            utility::idiom::pimpl<impl> m;
        public:
            explicit AudioFileInputArgs(std::unique_ptr<NUClear::Environment> environment);
    };
}
#endif

