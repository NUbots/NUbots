/*
 * This file is part of DanceEngine.
 *
 * DanceEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DanceEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DanceEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_DANCEENGINE_H
#define MODULES_DANCEENGINE_H

#include <NUClear.h>
#include "messages/Script.h"

namespace modules {

    class DanceEngine : public NUClear::Reactor {
    private:
        std::map<std::string, messages::Script> scripts;
    public:
        explicit DanceEngine(NUClear::PowerPlant* plant);
    };
}
#endif

