/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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
#include "Encoder.hpp"

namespace utility::nbs {

    Encoder::Encoder(std::filesystem::path path) : output_file(path), index_file(path += ".idx") {}

    Encoder::Encoder(const std::filesystem::path& path, const std::filesystem::path& index_path)
        : output_file(path), index_file(index_path) {}

    int Encoder::write(const NUClear::clock::time_point& timestamp,
                       const uint64_t& message_timestamp,
                       const uint64_t& hash,
                       const uint32_t& id,
                       const std::vector<uint8_t>& data) {
        // NBS File Format
        // Name      | Type                  |  Description
        // ------------------------------------------------------------
        // header    | char[3]               | NBS packet header ☢ { 0xE2, 0x98, 0xA2 }
        // length    | uint32_t              | Length of this packet after this value
        // timestamp | uint64_t              | Timestamp the data was emitted in microseconds
        // hash      | uint64_t              | the 64bit hash for the payload type
        // payload   | uint8_t[length - 16]  | the data payload

        // Convert the timestamp to a 64bit microsecond timestamp
        uint64_t timestamp_us =
            std::chrono::duration_cast<std::chrono::duration<uint64_t, std::micro>>(timestamp.time_since_epoch())
                .count();

        // The size of our output timestamp hash and data
        uint32_t size = data.size() + sizeof(hash) + sizeof(timestamp_us);

        // Write radiation symbol
        output_file.put(char(0xE2));
        output_file.put(char(0x98));
        output_file.put(char(0xA2));

        // Write the size of the packet
        output_file.write(reinterpret_cast<const char*>(&size), sizeof(size));

        // Write the timestamp
        output_file.write(reinterpret_cast<const char*>(&timestamp_us), sizeof(timestamp_us));

        // Write the hash
        output_file.write(reinterpret_cast<const char*>(&hash), sizeof(hash));

        // Write the actual packet data
        output_file.write(reinterpret_cast<const char*>(data.data()), data.size());
        output_file.flush();

        // NBS Index File Format
        // Name      | Type               |  Description
        // ------------------------------------------------------------
        // hash      | uint64_t           | the 64bit hash for the payload type
        // id        | uint32_t           | the id field of the payload
        // timestamp | uint64_t           | Timestamp of the message or the emit timestamp in nanoseconds
        // offset    | uint64_t           | offset to start of radiation symbol ☢
        // size      | uint32_t           | Size of the whole packet from the radiation symbol

        // Calculate the NBS Packets full size
        uint32_t full_size = HEADER_SIZE + sizeof(size) + size;

        index_file.write(reinterpret_cast<const char*>(&hash), sizeof(hash));
        index_file.write(reinterpret_cast<const char*>(&id), sizeof(id));
        index_file.write(reinterpret_cast<const char*>(&message_timestamp), sizeof(message_timestamp));
        index_file.write(reinterpret_cast<const char*>(&bytes_written), sizeof(bytes_written));
        index_file.write(reinterpret_cast<const char*>(&full_size), sizeof(full_size));
        index_file.flush();

        // Update the number of bytes we have written to the nbs file
        bytes_written += full_size;

        return bytes_written;
    }

    const uint64_t& Encoder::get_bytes_written() const {
        return bytes_written;
    }

    void Encoder::close() {
        if (output_file.is_open()) {
            output_file.close();
        }
        if (index_file.is_open()) {
            index_file.close();
        }
    }

    void Encoder::open(const std::filesystem::path& path) {
        output_file.open(path);
        index_file.open(path);
    }

    bool Encoder::is_open() const {
        return output_file.is_open() && index_file.is_open();
    }

}  // namespace utility::nbs
