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
#include <fmt/format.h>

#include "HardwareIO.hpp"

namespace module::platform::OpenCR {

    NUClear::threading::ReactionHandle HardwareIO::create_model_watchdog() {
        return on<Watchdog<ModelWatchdog, 500, std::chrono::milliseconds>, Single, Pool<HardwareIO>>().then([this] {
            log<NUClear::WARN>(fmt::format("OpenCR model information not received, restarting system"));
            // Clear all packet queues just in case
            queue_clear_all();
            // Restart the system and exit the watchdog
            startup();
            return;
        });
    }

    NUClear::threading::ReactionHandle HardwareIO::create_packet_watchdog() {
        return on<Watchdog<PacketWatchdog, 20, std::chrono::milliseconds>, Single, Pool<HardwareIO>>().then([this] {
            // This is a hacky fix because the watchdog is not disabled quickly enough at the beginning.
            // This may be related to the out of order packets with Sync within NUClear. This should be
            // fixed in a later version of NUClear.
            if (model_watchdog.enabled()) {
                log<NUClear::WARN>(
                    "Packet watchdog cannot be enabled while model watchdog is enabled. You may see this "
                    "warning at the start of the program. This is expected as the watchdog reaction may "
                    "still be disabling.");
                packet_watchdog.disable();
                return;
            }

            // Check what the hangup was

            int num_packets_dropped        = 0;                 // keep track of how many we dropped
            NUgus::ID first_dropped_packet = NUgus::ID::NO_ID;  // keep track in case we have a chain

            // The result of the assignment is 0 (NUgus::ID::NO_ID) if we aren't waiting on
            // any packets, otherwise is the nonzero ID of the timed out device
            for (NUgus::ID dropout_id; (dropout_id = queue_item_waiting()) != NUgus::ID::NO_ID;) {

                // Delete the packet we're waiting on
                packet_queue[dropout_id].erase(packet_queue[dropout_id].begin());

                // notify with ID and servo name
                log<NUClear::WARN>(
                    fmt::format("Dropped packet from ID {} ({})", int(dropout_id), nugus.device_name(dropout_id)));

                // if this is the first packet, set our flag
                if (num_packets_dropped == 0) {
                    first_dropped_packet = dropout_id;
                }

                // increment our counter
                num_packets_dropped++;
            }

            // if this is the first packet then send a warning
            if (num_packets_dropped > 1) {
                log<NUClear::WARN>(
                    fmt::format("NOTE: A dropped response packet by a dynamixel device in a SYNC READ/WRITE "
                                "chain will cause all later packets (of higher ID) to be dropped. Consider "
                                "checking cables for ID {} ({})",
                                int(first_dropped_packet),
                                nugus.device_name(first_dropped_packet)));
            }

            // Send a request for all servo packets, only if there were packets dropped
            // In case the system stops for some other reason, we don't want the watchdog
            // to make it automatically restart
            if (num_packets_dropped > 0) {
                log<NUClear::WARN>("Flushing buffer and requesting servo packets to restart system");
                // We *may* have data that we haven't gotten around to processing by the time the watchdog triggers.
                // if this is a full packet (somehow) it will throw us out of sync unless we flush the buffer.
                opencr.flush();
                send_servo_request();
            }
        });
    }

}  // namespace module::platform::OpenCR
