/*
 * This file is part of Darwin Hardware IO.
 *
 * Darwin Hardware IO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Hardware IO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Hardware IO.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "MLNCH.h"

namespace Darwin {

    MLNCH::MLNCH(UART& coms, int id) : DarwinDevice(coms, id) {}

    void MLNCH::fire() {
        std::vector<uint8_t> command({0xFF, 0xFF, 51, 2, 3, 0x00});
        //coms.calculateChecksum(command.data());
        coms.executeBroadcast(command);
    }
} // Darwin