/*
 * This file is part of NUbugger.
 *
 * NUbugger is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUbugger is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUbugger.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_DEBUG_NUBUGGER_H
#define MODULES_DEBUG_NUBUGGER_H

#include <nuclear>

namespace modules {
namespace debug {

    class NUbugger : public NUClear::Reactor {
    public:
        explicit NUbugger(std::unique_ptr<NUClear::Environment> environment);
    };
    
}  // debug
}  // modules

#endif  // MODULES_SUPPORT_NUBUGGER_H

