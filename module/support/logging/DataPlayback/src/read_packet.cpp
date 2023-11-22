/*
 * MIT License
 *
 * Copyright (c) 2017 NUbots
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
#include "read_packet.hpp"

#include <iostream>
namespace module::support::logging {

    Packet read_packet(std::ifstream& in) {

        // Try to lock onto the next header in the stream
        int header_lock = 0;
        while (header_lock != 3) {
            int r = in.get();
            switch (r) {
                case 0xE2: header_lock = 1; break;
                case 0x98: header_lock = header_lock == 1 ? 2 : 0; break;
                case 0xA2: header_lock = header_lock == 2 ? 3 : 0; break;
                default: header_lock = 0; break;
                case -1: throw std::runtime_error("End of file");
            }
        }

        // We are header locked! read our length
        uint32_t length = 0;
        in.read(reinterpret_cast<char*>(&length), sizeof(length));

        uint64_t timecode = 0;
        in.read(reinterpret_cast<char*>(&timecode), sizeof(timecode));

        // Build our packet!
        Packet p;
        p.timecode = std::chrono::microseconds(timecode);
        in.read(reinterpret_cast<char*>(&p.hash), sizeof(p.hash));
        p.payload.resize(length - sizeof(p.hash) - sizeof(p.timecode));
        in.read(p.payload.data(), p.payload.size());

        return p;
    }
}  // namespace module::support::logging
