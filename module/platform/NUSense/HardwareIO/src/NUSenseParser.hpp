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
#ifndef MODULE_PLATFORM_NUSENSE_HARDWAREIO_NUSENSE_PARSER_HPP
#define MODULE_PLATFORM_NUSENSE_HARDWAREIO_NUSENSE_PARSER_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace module::platform::NUSense {

    struct NUSenseFrame {
        /// NBS Header, should contain 0xE2, 0x98, 0xA2
        std::array<uint8_t, 3> header{};
        /// Size of the payload after this field
        uint32_t size;
        /// Timestamp of the message
        uint64_t timestamp;
        /// Hash of the message type
        uint64_t hash;
        /// The payload of the message
        std::vector<uint8_t> payload{};
    };

    struct NUSenseParser {

        /// @brief Parses bytes from NUSense one byte at a time and returns a NUSenseFrame when a complete frame is read
        /// @param byte The next byte to parse
        /// @return A NUSenseFrame if a complete frame has been read, otherwise nullptr
        std::unique_ptr<NUSenseFrame> operator()(const uint8_t& byte);

    private:
        /// States for the parsers state machine
        enum State { INITIAL, HEADER_1, HEADER_2, PAYLOAD, SIZE } state = INITIAL;
        /// Number of bytes to read after the size field
        uint32_t size;
        /// Buffer to store bytes read so far
        std::vector<uint8_t> buffer;
    };

}  // namespace module::platform::NUSense

#endif  // MODULE_PLATFORM_NUSENSE_HARDWAREIO_NUSENSE_PARSER_HPP
