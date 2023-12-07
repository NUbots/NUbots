/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "HardwareIO.hpp"

namespace module::platform::OpenCR {

    /**
     * @todo make a PacketQueue class with these as memeber functions and add wrappers
     * for queue.push() and queue.pop() to clean everything up.
     */

    NUgus::ID HardwareIO::servo_waiting() {
        // Loop through all servos in order
        for (const auto& id : nugus.servo_ids()) {
            // If the length is non-zero then return
            if (!packet_queue[NUgus::ID(id)].empty()) {
                return NUgus::ID(id);
            }
        }
        // no servos waiting
        return NUgus::ID::NO_ID;
    }

    bool HardwareIO::opencr_waiting() {
        // just a basic wrapper for consistency
        return !packet_queue[NUgus::ID::OPENCR].empty();
    }

    NUgus::ID HardwareIO::queue_item_waiting() {
        // Loop through all devices in the queue
        for (const auto& keypair : packet_queue) {
            // If any are waiting, then return their ID
            if (keypair.second.size()) {
                return keypair.first;
            }
        }
        // no devices waiting
        return NUgus::ID::NO_ID;
    }

    int HardwareIO::queue_clear_all() {
        int packets_cleared = 0;
        // Loop through all initialised queues
        for (const auto& keypair : packet_queue) {
            // Add the packets to the running total
            packets_cleared += keypair.second.size();
            // Clear the queue
            packet_queue[keypair.first].clear();
        }
        return packets_cleared;
    }

}  // namespace module::platform::OpenCR
