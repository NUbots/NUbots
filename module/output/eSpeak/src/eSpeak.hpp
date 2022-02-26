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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_OUTPUT_ESPEAK_HPP
#define MODULES_OUTPUT_ESPEAK_HPP

#include <nuclear>

namespace module::output {

    /**
     * Takes strings given to it and plays them using text to speech
     *
     * @author Trent Houliston
     */
    class eSpeak : public NUClear::Reactor {
    public:
        explicit eSpeak(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::output

#endif  // MODULES_OUTPUT_ESPEAK_HPP
