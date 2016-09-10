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

#ifndef MODULES_INPUT_AUDIOFILEINPUT_H
#define MODULES_INPUT_AUDIOFILEINPUT_H

#include <nuclear>
#include "utility/idiom/pimpl.h"

namespace module {
    namespace input {

        /**
         * Reads audio from a WAV file and plays it in real time as if hearing it.
         *
         * @author Joshua Kearns
         * @author Trent Houliston
         */
        class AudioFileInput : public NUClear::Reactor {
            private:
                class impl;
                utility::idiom::pimpl<impl> m;
            public:
                explicit AudioFileInput(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // audio
}  // modules

#endif  // MODULES_INPUT_AUDIOFILEINPUT_H
