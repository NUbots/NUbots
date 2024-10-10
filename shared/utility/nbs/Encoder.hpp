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
#ifndef UTILITY_NBS_ENCODER_HPP
#define UTILITY_NBS_ENCODER_HPP

#include <filesystem>
#include <fstream>
#include <nuclear>
#include <zstr.hpp>

namespace utility::nbs {

    class Encoder {
    private:
        /// The size of the radiation symbol at the start of each packet
        static constexpr int HEADER_SIZE = 3;
        /// The file we are outputting to currently
        std::ofstream output_file{};
        /// The file we are outputting our index to currently
        zstr::ofstream index_file{};
        /// The number of bytes written to the nbs file
        uint64_t bytes_written = 0;

    public:
        Encoder() = default;

        Encoder(std::filesystem::path path);

        Encoder(const std::filesystem::path& path, const std::filesystem::path& index_path);

        /// @brief Write a neutron to the file and returns the number of bytes written
        template <typename T>
        int write(const T& value, const NUClear::clock::time_point& timestamp) {
            return write(timestamp,
                         get_timestamp(value, timestamp),
                         NUClear::util::serialise::Serialise<T>::hash(),
                         get_id(value),
                         NUClear::util::serialise::Serialise<T>::serialise(value));
        }

        /**
         * @brief Writes an entry to the file
         *
         * @param timestamp the timestamp the message was originally emited at
         * @param message_timestamp the timestamp inside the message
         * @param hash The type hash of the message
         * @param id The id of the message
         * @param data The raw message data to be written
         */
        int write(const NUClear::clock::time_point& timestamp,
                  const uint64_t& message_timestamp,
                  const uint64_t& hash,
                  const uint32_t& id,
                  const std::vector<uint8_t>& data);

        /// @brief Gets the number of bytes written
        const uint64_t& get_bytes_written() const;

        /// @brief closes the internal nbs file
        void close();

        void open(const std::filesystem::path& path);

        /// @brief checks that the nbs file is open
        bool is_open() const;
    };

}  // namespace utility::nbs

#endif  // UTILITY_NBS_ENCODER_HPP
