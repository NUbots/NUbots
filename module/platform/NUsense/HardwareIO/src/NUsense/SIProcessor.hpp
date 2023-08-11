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

#ifndef NUSENSE_SIPROCESSOR_HPP
#define NUSENSE_SIPROCESSOR_HPP

#include "message/platform/RawSensors.hpp"

namespace module::platform::NUsense {
    using message::platform::RawSensors;

    class SIProcessor {
    public:
        // Protobuf msg to nbs packet
        std::vector<char> msg_to_nbs(const RawSensors& msg);

        // Nbs packet to protobuf msg
        const RawSensors& nbs_to_msg(std::vector<char> nbs_packet);
    };
}  // namespace module::platform::NUsense
#endif
