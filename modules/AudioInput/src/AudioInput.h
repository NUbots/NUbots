/*
 * This file is part of AudioInput.
 *
 * AudioInput is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * AudioInput is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with AudioInput.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_AUDIOINPUT_H
#define MODULES_AUDIOINPUT_H

#include <NUClear.h>
#include "utility/idiom/pimpl.h"

namespace modules {

    /**
     * TODO document
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
}
#endif

