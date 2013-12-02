/*
 * This file is part of eSpeakReactor.
 *
 * eSpeakReactor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * eSpeakReactor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with eSpeakReactor.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_OUTPUT_ESPEAK_H
#define MODULES_OUTPUT_ESPEAK_H

#include <nuclear>

namespace modules {
    namespace output {

        /**
         * Takes strings given to it and plays them using text to speech
         *
         * @author Trent Houliston
         */
        class eSpeak : public NUClear::Reactor {
        public:
            explicit eSpeak(std::unique_ptr<NUClear::Environment> environment);
        };
        
    }  // output
}  // modules

#endif  // MODULES_OUTPUT_ESPEAK_H

