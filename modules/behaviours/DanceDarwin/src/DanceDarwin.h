/*
 * This file is part of DanceDarwin.
 *
 * DanceDarwin is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DanceDarwin is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DanceDarwin.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_DANCEDARWIN_H
#define MODULES_DANCEDARWIN_H

#include <nuclear>
#include "messages/Script.h"

namespace modules {
    namespace Behaviours {
        class DanceDarwin : public NUClear::Reactor {
        private:
            std::map<std::string, messages::Script> scripts;
            bool startedDancing;
        public:
            explicit DanceDarwin(std::unique_ptr<NUClear::Environment> environment);
        };
    }
}
#endif

