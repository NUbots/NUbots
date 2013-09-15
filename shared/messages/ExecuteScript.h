/*
 * This file is part of ScriptEngine.
 *
 * ScriptEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_EXECUTESCRIPT_H
#define MESSAGES_EXECUTESCRIPT_H

#include <NUClear.h>

namespace messages {

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    struct ExecuteScript {
        ExecuteScript(const std::string& scriptName, NUClear::clock::time_point start = NUClear::clock::now()) : script(scriptName), start(start) {};
        std::string script;
        NUClear::clock::time_point start;
    };
}

#endif    /* MODULES_EXECUTESCRIPT_H */

