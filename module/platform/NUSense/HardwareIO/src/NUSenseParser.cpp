/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#include "NUSenseParser.hpp"

namespace module::platform::NUSense {
    /// @brief  Read a buffer of bytes and convert it to an unsigned 32 bit integer in little endian
    ///         HwIO uses this to read the size field of the nbs packet sent by NUSense
    /// @param  ptr Pointer to a buffer of bytes
    /// @return The unsigned 32 bit number decoded from the buffer of bytes
    uint32_t read_le_32(const uint8_t* ptr) {
        return (uint32_t(ptr[0]) << 0) | (uint32_t(ptr[1]) << 8) | (uint32_t(ptr[2]) << 16) | (uint32_t(ptr[3]) << 24);
    }

    /// @brief  Read a buffer of bytes and convert it to an unsigned 64 bit integer in little endian
    ///         HwIO uses this to read the message hash and timestamp fields of the nbs packet sent by NUSense
    /// @param  ptr Pointer to a buffer of bytes
    /// @return The unsigned 64 bit number decoded from the buffer of bytes
    uint64_t read_le_64(const uint8_t* ptr) {
        return (uint64_t(ptr[0]) << 0) | (uint64_t(ptr[1]) << 8) | (uint64_t(ptr[2]) << 16) | (uint64_t(ptr[3]) << 24)
               | (uint64_t(ptr[4]) << 32) | (uint64_t(ptr[5]) << 40) | (uint64_t(ptr[6]) << 48)
               | (uint64_t(ptr[7]) << 56);
    }

    std::unique_ptr<NUSenseFrame> NUSenseParser::operator()(const uint8_t& byte) {
        if (state == INITIAL) {
            buffer.clear();
        }

        buffer.push_back(byte);
        switch (state) {
            case INITIAL: state = byte == 0xE2 ? HEADER_1 : INITIAL; break;
            case HEADER_1: state = byte == 0x98 ? HEADER_2 : INITIAL; break;
            case HEADER_2: state = byte == 0xA2 ? SIZE : INITIAL; break;
            case SIZE: {
                // We have read the size of the payload
                if (buffer.size() == 7) {
                    size  = read_le_32(&buffer[3]);
                    state = PAYLOAD;
                }
            } break;
            case PAYLOAD: {
                // Read all the bytes of this packet
                if (buffer.size() == size + 3 + sizeof(size)) {
                    // We always go back to initial after a payload
                    state = INITIAL;

                    auto msg = std::make_unique<NUSenseFrame>();

                    msg->header = {buffer[0], buffer[1], buffer[2]};
                    msg->size   = size;

                    // Accessor variable initialised to 3 bytes of header + 4 bytes of size, since it is uint32_t
                    uint32_t buf_idx = sizeof(msg->header) + sizeof(size);
                    msg->timestamp   = read_le_64(&buffer[buf_idx]);

                    // Add 8 bytes to the accessor variable to account for accessing the message timestamp above
                    buf_idx += sizeof(msg->timestamp);
                    msg->hash = read_le_64(&buffer[buf_idx]);

                    // Add another 8 bytes, updating the accessor variable to account for accessing the message type
                    // hash
                    buf_idx += sizeof(msg->hash);
                    msg->payload = std::vector<uint8_t>(std::next(buffer.begin(), buf_idx), buffer.end());

                    return msg;
                }
            }
        }

        return nullptr;
    }

}  // namespace module::platform::NUSense
