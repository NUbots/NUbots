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

    using message::platform::StatusReturn;

    enum Phases : uint8_t {
        IDLE,         // Wait for the first byte of the header field
        HEADER_SYNC,  // Match the next 3 bytes of the header after matching the first byte
        PREAMBLE,     // Look for the next 5 bytes after successfully locating the header
        DATA,         // Reading in the data
        UNSTUFF_1,    // Seen a 0xFF while reading
        UNSTUFF_2,    // Seen 0xFFFF while reading
        UNSTUFF_3,    // Seen 0xFFFFFD while reading, if next byte is 0xFD drop it
        FINISH,       // Finished reading a packet
        TIMEOUT       // Happens when packet_start_time + timeout is greater than the current time
    };

    void HardwareIO::handle_response() {
        static constexpr uint8_t packet_header[4]           = {0xFF, 0xFF, 0xFD, 0x00};
        static Phases current_phase                         = Phases::IDLE;
        static uint8_t sync_point                           = 0;
        static NUClear::clock::time_point packet_start_time = NUClear::clock::now();
        static std::chrono::microseconds timeout            = std::chrono::microseconds(packet_wait);
        static std::vector<uint8_t> response;
        static uint16_t packet_length = 0;

        // Read from the OpenCR device
        std::array<uint8_t, 128> buf;
        uint8_t num_bytes;
        num_bytes = opencr.read(buf.data(), 128);

        // Emits a completed StatusReturn message
        auto emit_msg = [&]() -> void {
            auto msg = std::make_unique<StatusReturn>();

            msg->magic       = 0x00FDFFFF;  // little endian header bytes
            msg->id          = response[4];
            msg->length      = packet_length;
            msg->instruction = response[7];

            // Extract alert flag from byte & cast the rest to CommandError
            msg->error = response[8];

            // Param field starts 9 bytes after start, CRC takes up last 2
            std::copy(response.begin() + 9, response.end() - 2, std::back_inserter(msg->data));

            msg->checksum  = (response[response.size() - 1] << 8) | response[response.size() - 2];
            msg->timestamp = NUClear::clock::now();

            // Check CRC
            if (dynamixel::v2::calculate_checksum(response) != msg->checksum) {
                log<NUClear::WARN>("Invalid CRC detected.");
                return;  // without emit
            }

            // Check we're emitting a Status (return) packet
            if ((uint8_t) msg->instruction != dynamixel::v2::Instruction::STATUS_RETURN) {
                log<NUClear::WARN>("Attempt to emit non Status(return) packet");
                return;  // without emit
            }

            // Emit if there are no errors
            emit(msg);
        };

        // Reset the state machine state
        auto reset_state = [&]() -> void {
            sync_point    = 0;
            packet_length = 0;
            current_phase = Phases::IDLE;
            response.clear();
        };

        // Process the data we have received
        for (uint8_t i = 0; i < num_bytes; ++i) {
            // Act based on the current phase of the state machine
            switch (current_phase) {
                // Idle phase
                // We haven't seen any data, so we are waiting for the
                // first byte of the header
                case Phases::IDLE:
                    // If we match the first byte of the header then
                    // transition to the HEADER_SYNC phase
                    if (packet_header[sync_point] == buf[i]) {
                        response.push_back(packet_header[sync_point]);
                        sync_point++;
                        current_phase     = Phases::HEADER_SYNC;
                        packet_start_time = NUClear::clock::now();
                        timeout           = std::chrono::microseconds(packet_wait);
                    }
                    break;

                // Header Sync phase
                // We have matched the first byte of the header
                // now match the next three bytes
                case Phases::HEADER_SYNC:
                    if (NUClear::clock::now() < (packet_start_time + timeout)) {
                        if (packet_header[sync_point] == buf[i]) {
                            response.push_back(packet_header[sync_point]);
                            sync_point++;
                        }

                        // Header has been matched
                        // Now read in the rest of the packet
                        if (sync_point == 4) {
                            sync_point        = 0;
                            current_phase     = Phases::PREAMBLE;
                            packet_start_time = NUClear::clock::now();
                            timeout           = std::chrono::microseconds(byte_wait * 5 + 2000 + packet_wait);
                        }
                    }
                    else {
                        current_phase = Phases::TIMEOUT;
                    }
                    break;

                // Preamble phase
                // We have the full header, now we are looking for the next 5 bytes
                // Packet ID, Length, Instruction ID, and Error
                case Phases::PREAMBLE:
                    if (NUClear::clock::now() < (packet_start_time + timeout)) {
                        response.push_back(buf[i]);

                        // We just read in the expected length of the packet
                        if (response.size() == 7) {
                            packet_length = (response[6] << 8) | response[5];
                        }

                        // We now have the header and the packet preamble
                        // Time to get the packet parameters and CRC
                        if (response.size() == 9) {
                            current_phase = Phases::DATA;
                            timeout       = std::chrono::microseconds(byte_wait * packet_length + 2000 + packet_wait);
                        }
                    }
                    else {
                        current_phase = Phases::TIMEOUT;
                    }
                    break;

                // Data phase
                // Header and preamble have been received
                // Now we read in the message parameters and CRC
                // We should be looking for (packet_length - 2) bytes
                case Phases::DATA:
                    if (NUClear::clock::now() < (packet_start_time + timeout)) {
                        response.push_back(buf[i]);

                        // We now have all of our data
                        if (response.size() == size_t(7 + packet_length)) {
                            current_phase = Phases::FINISH;
                        }
                        // Go to UNSTUFF_1 if we see 0xFF in the data field and we haven't finished yet
                        else if (buf[i] == packet_header[0]) {
                            current_phase = Phases::UNSTUFF_1;
                        }
                    }
                    else {
                        current_phase = Phases::TIMEOUT;
                    }
                    break;

                // Going to the case below means that we saw a 0xFF in the data field
                // If the next byte is another 0xFF, then we go to UNSTUFF_2
                case Phases::UNSTUFF_1:
                    current_phase = buf[i] == packet_header[1] ? Phases::UNSTUFF_2 : Phases::DATA;
                    response.push_back(buf[i]);

                    // In case we are finished with the packet
                    if (response.size() == size_t(7 + packet_length)) {
                        current_phase = Phases::FINISH;
                    }
                    break;

                // Going to the case below means that we've seen 0xFFFF in the data field
                // If the next byte is the 0xFD, then we go to UNSTUFF_3
                case Phases::UNSTUFF_2:
                    current_phase = buf[i] == packet_header[2] ? Phases::UNSTUFF_3 : Phases::DATA;
                    response.push_back(buf[i]);

                    // In case we are finished with the packet
                    if (response.size() == size_t(7 + packet_length)) {
                        current_phase = Phases::FINISH;
                    }
                    break;

                // Going to the case below means that we've seen our 0xFFFFFD in the data field
                // If the next byte is 0xFD, ignore it and go back to the DATA phase
                case Phases::UNSTUFF_3:
                    if (buf[i] != packet_header[2]) {
                        response.push_back(buf[i]);
                    }
                    // If we decide to ignore a packet we have to decrement the packet length
                    else {
                        packet_length--;
                    }

                    current_phase = Phases::DATA;

                    // In case we are finished with the packet
                    if (response.size() == size_t(7 + packet_length)) {
                        current_phase = Phases::FINISH;
                    }
                    break;

                // Finish phase
                // We have now received a complete message
                // However, it looks like we have more data to process
                // Package up the current message, and reset our buf counter and phase
                case Phases::FINISH:
                    emit_msg();

                    // Set up for next message
                    i--;  // Decrement counter to account for the next increment
                    reset_state();
                    break;

                // Timeout phase
                // It took too long to read in the full packet .... Giving up
                case Phases::TIMEOUT:
                    log<NUClear::WARN>("Packet timeout occurred.");
                    reset_state();
                    break;

                // Yea, dont know what happened here
                default: reset_state(); break;
            }
        }

        // Our input buffer ended at the exact end of the packet
        if (response.size() == size_t(7 + packet_length)) {
            emit_msg();
            reset_state();
        }
    }

}  // namespace module::platform::OpenCR
