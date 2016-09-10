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

#ifndef MODULES_INPUT_AUDIOINPUT_H
#define MODULES_INPUT_AUDIOINPUT_H

#include <nuclear>
#include "utility/idiom/pimpl.h"

namespace module {
    namespace input {

        /**
         * Uses the microphone in the system to provide a constant stream of audio
         *
         * @author Jake Woods
         */
        class AudioInput : public NUClear::Reactor {
            public:
                explicit AudioInput(std::unique_ptr<NUClear::Environment> environment);
            private:
                class impl;
                utility::idiom::pimpl<impl> m;
        };

    }  // input
}  // modules

#endif  // MODULES_INPUT_AUDIOINPUT_H
