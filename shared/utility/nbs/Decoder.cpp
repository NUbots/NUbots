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
#include "Decoder.hpp"

#include <chrono>
#include <filesystem>
#include <functional>
#include <map>
#include <mio/mmap.hpp>
#include <nuclear>
#include <numeric>
#include <vector>

#include "Index.hpp"

namespace utility::nbs {

    Decoder::Decoder(const std::vector<std::filesystem::path>& paths, const bool& show_progress)
        : index(paths, Index::Ordering::TIME, show_progress) {

        // Memory map all the files to access the data
        for (const auto& path : paths) {
            // Memory map the file into ram
            mmaps.emplace_back(path.string());
        }
    }

    Decoder::Decoder(const std::filesystem::path& path, const bool& show_progress)
        : Decoder(std::vector<std::filesystem::path>({path}), show_progress) {}

    void Decoder::process(
        const std::function<void(const uint64_t&, const uint64_t&, const uint64_t&, const uint64_t&)>& progress) {

        // Work out the total number of bytes to be handled from all the files
        uint64_t total_bytes =
            std::accumulate(mmaps.begin(),
                            mmaps.end(),
                            uint64_t(0),
                            [&](const uint64_t& a, const mio::ummap_source& b) { return a + b.size(); });
        uint64_t total_messages = std::distance(index.begin(), index.end());

        // Loop through the index
        uint64_t current_bytes   = 0;
        uint64_t current_message = 0;

        for (const auto& e : *this) {
            if (e.has_callback()) {
                e();
            }

            // Update the progress callback
            current_bytes += e.item->item.length;
            current_message += 1;
            if (progress) {
                progress(current_message, total_messages, current_bytes, total_bytes);
            }
        }
    }

    void Decoder::unhandled(const std::function<void(const uint64_t& emit_timestamp,
                                                     const uint64_t& index_timestamp,
                                                     const uint64_t& hash_type,
                                                     const uint32_t& subtype,
                                                     const void* payload,
                                                     const size_t& payload_length)>& callback) {
        unhandled_callback = callback;
    }

    void Decoder::all(const std::function<void(const uint64_t& emit_timestamp,
                                               const uint64_t& index_timestamp,
                                               const uint64_t& hash_type,
                                               const uint32_t& subtype,
                                               const void* payload,
                                               const size_t& payload_length)>& callback) {
        all_callback = callback;
    }


}  // namespace utility::nbs
