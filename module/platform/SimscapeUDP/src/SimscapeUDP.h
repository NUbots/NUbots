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
 * Copyright 2020 NUbots <nubots@nubots.net>
 */


#ifndef MODULE_PLATFORM_SimscapeUDP_H
#define MODULE_PLATFORM_SimscapeUDP_H

#include <nuclear>

namespace module {
namespace platform {

    class SimscapeUDP : public NUClear::Reactor {
    public:
        explicit SimscapeUDP(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct {
            std::string ip;
            uint inPort;
            uint outPort;
        } config;

        int test = 0;
    };

}  // namespace platform
}  // namespace module

#endif
