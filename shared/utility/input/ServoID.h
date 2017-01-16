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

#ifndef UTILITY_INPUT_SERVOID_H
#define UTILITY_INPUT_SERVOID_H

#include <string>

#include "message/input/proto/Sensors.h"

namespace utility {
    namespace input {

        enum ServoSide {
            RIGHT = 0,
            LEFT = 1
        };

        const std::string stringFromId(const message::input::proto::Sensors::ServoID::Value& id);
        message::input::proto::Sensors::ServoID::Value idFromString(const std::string& str);
        message::input::proto::Sensors::ServoID::Value idFromPartialString(const std::string& str, const ServoSide& side);
    }
}

#endif