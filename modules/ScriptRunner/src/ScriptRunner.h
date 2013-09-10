/*
 * This file is part of ScriptRunner.
 *
 * ScriptRunner is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ScriptRunner is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with ScriptRunner.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Jake Woods <jake.f.woods@gmail.com>
 */

#ifndef MODULES_SCRIPTRUNNER_H
#define MODULES_SCRIPTRUNNER_H

#include <queue>

#include <NUClear.h>

#include "messages/DarwinServoCommand.h"
#include "messages/Script.h"

namespace modules {

    class ScriptRunner : public NUClear::Reactor {
    private:
        std::queue<std::string> scripts;

        void executeNextScript();
    public:
        explicit ScriptRunner(NUClear::PowerPlant* plant);
    };
}
#endif

