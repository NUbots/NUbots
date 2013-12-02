/*
 * This file is part of PartyDarwin.
 *
 * PartyDarwin is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PartyDarwin is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PartyDarwin.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_PLATFORM_DARWIN_PARTYDARWIN_H
#define MODULES_PLATFORM_DARWIN_PARTYDARWIN_H

#include <nuclear>

namespace modules {
    namespace platform {
        namespace darwin {

            /**
             * Makes the Darwin's eyes flash random colours, useful for debugging.
             *
             * @author Trent Houliston
             */
            class PartyDarwin : public NUClear::Reactor {
            public:
                explicit PartyDarwin(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // darwin
    }  // platform
}  // modules

#endif  // MODULES_PLATFORM_DARWIN_PARTYDARWIN_H

