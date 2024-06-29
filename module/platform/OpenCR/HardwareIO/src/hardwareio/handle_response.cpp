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

    static constexpr uint8_t packet_header[4] = {0xFF, 0xFF, 0xFD, 0x00};
    static Phases current_phase               = Phases::IDLE;
    static uint8_t sync_point                 = 0;
    static uint16_t payload_length            = 0;
    static uint16_t crc                       = 0x00;

    static std::vector<uint8_t> response;

    static inline void reset_state() {
        sync_point     = 0;
        payload_length = 0;
        current_phase  = Phases::IDLE;
        crc            = 0x00;
        response.clear();
    }

    void HardwareIO::emit_status_return() {
        auto msg = std::make_unique<StatusReturn>();

        msg->magic       = 0x00FDFFFF;  // little endian header bytes
        msg->id          = response[4];
        msg->length      = payload_length;
        msg->instruction = response[7];

        // Extract alert flag from byte & cast the rest to CommandError
        msg->error = response[8];

        // Param field starts 9 bytes after start, CRC takes up last 2
        std::copy(response.begin() + 9, response.end() - 2, std::back_inserter(msg->data));

        msg->checksum  = (response[response.size() - 1] << 8) | response[response.size() - 2];
        msg->timestamp = NUClear::clock::now();

        // Check CRC
        if (crc != msg->checksum) {
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
    }

    void HardwareIO::handle_response() {
        static NUClear::clock::time_point packet_start_time = NUClear::clock::now();
        static std::chrono::microseconds timeout            = std::chrono::microseconds(packet_wait);

        // Read from the OpenCR device
        std::array<uint8_t, 128> buf;
        uint8_t num_bytes;
        num_bytes = opencr.read(buf.data(), 128);

        // Process the data we have received
        for (uint8_t i = 0; i < num_bytes; ++i) {

            uint8_t byte = buf[i];

            // Act based on the current phase of the state machine
            switch (current_phase) {
                // Idle phase
                // We haven't seen any data, so we are waiting for the
                // first byte of the header
                case Phases::IDLE:
                    // If we match the first byte of the header then
                    // transition to the HEADER_SYNC phase
                    if (packet_header[sync_point] == byte) {
                        response.push_back(byte);
                        crc = dynamixel::v2::update_crc(crc, byte);
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
                        if (packet_header[sync_point] == byte) {
                            response.push_back(byte);
                            crc = dynamixel::v2::update_crc(crc, byte);
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
                        response.push_back(byte);
                        crc = dynamixel::v2::update_crc(crc, byte);

                        // We just read in the expected length of the packet
                        if (response.size() == 7) {
                            payload_length = (response[6] << 8) | response[5];
                        }

                        // We now have the header and the packet preamble
                        // Time to get the packet parameters and CRC
                        if (response.size() == 9) {
                            current_phase = Phases::DATA;
                            timeout       = std::chrono::microseconds(byte_wait * payload_length + 2000 + packet_wait);
                        }
                    }
                    else {
                        current_phase = Phases::TIMEOUT;
                    }
                    break;

                case Phases::DATA:
                case Phases::UNSTUFF_1:
                case Phases::UNSTUFF_2:
                    if (NUClear::clock::now() < (packet_start_time + timeout)) {

                        response.push_back(byte);

                        // Track system state while the response is less than the expected payload length which is
                        // 4 bytes for the header, 1 byte for the ID, 2 bytes for the length, and the payload_length
                        // -2 bytes for the CRC
                        if (static_cast<uint16_t>(response.size()) <= 4 + 1 + 2 + payload_length - 2) {
                            // Update crc while reading data (don't include the crc bytes in the crc calculation)
                            crc = dynamixel::v2::update_crc(crc, byte);

                            switch (current_phase) {
                                case Phases::DATA:
                                    current_phase = byte == 0xFF ? Phases::UNSTUFF_1 : Phases::DATA;
                                    break;
                                case Phases::UNSTUFF_1:
                                    current_phase = byte == 0xFF ? Phases::UNSTUFF_2 : Phases::DATA;
                                    break;
                                case Phases::UNSTUFF_2:
                                    current_phase = byte == 0xFD ? Phases::UNSTUFF_3 : Phases::DATA;
                                    break;
                                default: break;
                            }
                        }

                        // In case we are finished with the packet
                        if (static_cast<uint16_t>(response.size()) == size_t(4 + 1 + 2 + payload_length)) {
                            current_phase = Phases::FINISH;

                            uint16_t unstuffed_size = payload_length - 7;
                            response[5]             = unstuffed_size & 0xFF;
                            response[6]             = unstuffed_size >> 8;
                        }
                        break;
                    }
                    else {
                        current_phase = Phases::TIMEOUT;
                    }
                    break;

                // Going to the case below means that we've seen our 0xFFFFFD in the data field
                // If the next byte is 0xFD, ignore it and go back to the DATA phase
                case Phases::UNSTUFF_3:
                    if (NUClear::clock::now() < (packet_start_time + timeout)) {
                        switch (byte) {
                            case 0xFD:
                                --payload_length;
                                crc = dynamixel::v2::update_crc(crc, byte);  // We will still have to crc the stuffing
                                current_phase = Phases::DATA;
                                break;

                            // This is somehow another header within the data field - if the size of the response is
                            // less than 5 (meaning it only has the header inside it again) then we can still go back to
                            // PREAMBLE. If this is not the case then we have to reset
                            case 0x00:
                                if (response.size() < 4 + 1) {
                                    current_phase = Phases::PREAMBLE;
                                }
                                else {
                                    reset_state();
                                }
                                break;

                            // We have no clue what happened so just miss some packets and start again
                            default: reset_state(); break;
                        }
                    }
                    else {
                        current_phase = Phases::TIMEOUT;
                    }
                    break;

                // Finish phase
                // We have now received a complete message
                // However, it looks like we have more data to process
                // Package up the current message, and reset our buf counter and phase
                case Phases::FINISH:
                    emit_status_return();

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
        if (response.size() == size_t(7 + payload_length)) {
            emit_status_return();
            reset_state();
        }
    }

}  // namespace module::platform::OpenCR
