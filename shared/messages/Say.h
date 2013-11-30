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

#ifndef MESSAGES_SAY_H
#define MESSAGES_SAY_H

#include <string>

namespace messages {

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    struct Say : public std::string {
        Say(std::string message) : std::string(message) {};
    };
};

#endif